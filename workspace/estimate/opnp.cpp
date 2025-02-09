#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <vector>
#include <iostream>

#include "estimators/utils.h"

#include "estimate/opnp.h"
#include <cmath>
#include <algorithm>

// Define the static member variable
OPnPEstimator::TemplateData OPnPEstimator::template_data_;

std::vector<OPnPEstimator::M_t> OPnPEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
    std::vector<M_t> models;

    if(points2D.size() < kMinNumSamples || points3D.size() < kMinNumSamples) {
        return models;
    }

    OPnPEstimator opnp;
    M_t proj_matrix;

    if (!opnp.ComputeOPnPPose(points2D, points3D, &proj_matrix)) {
        return std::vector<OPnPEstimator::M_t>({});
    }

    return std::vector<OPnPEstimator::M_t>({proj_matrix});
}

void OPnPEstimator::Residuals(const std::vector<X_t>& points2D,
                              const std::vector<Y_t>& points3D,
                              const M_t& proj_matrix, std::vector<double>* residuals) {
    residuals->clear();
    colmap::ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}

bool OPnPEstimator::ComputeOPnPPose(const std::vector<Eigen::Vector2d>& points2D,
                                    const std::vector<Eigen::Vector3d>& points3D,
                                    Eigen::Matrix3x4d* proj_matrix) {
    if (points2D.size() != points3D.size() || points2D.size() < 4) {
        return false;
    }

    Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>> points3D_mat(points3D[0].data(), 3, points3D.size());


    getTemplateData();  // Ensures template is loaded
    SolverSettings settings;
    settings.template_data = template_data_;

    const int n = points2D.size();

    // Calculate centroid (mean(U,2) in MATLAB)
    Eigen::Vector3d Ucent = Eigen::Vector3d::Zero();
    for (const auto& p : points3D) {
        Ucent += p;
    }
    Ucent /= n;
    
    // Center the points (Um = U - repmat(Ucent,1,n))
    std::vector<Eigen::Vector3d> Um(n);
    std::vector<double> xm(n), ym(n), zm(n);  // Centered coordinates
    std::vector<double> x(n), y(n), z(n);     // Original coordinates
    
    for (int i = 0; i < n; ++i) {
        // Store original coordinates
        x[i] = points3D[i].x();
        y[i] = points3D[i].y();
        z[i] = points3D[i].z();
        
        // Center points
        Um[i] = points3D[i] - Ucent;
        xm[i] = Um[i].x();
        ym[i] = Um[i].y();
        zm[i] = Um[i].z();
    }
    
    // Store 2D points (similar to u1 = u(1,:)', v1 = u(2,:)')
    std::vector<double> u1(n), v1(n);
    for (int i = 0; i < n; ++i) {
        u1[i] = points2D[i].x();
        v1[i] = points2D[i].y();
    }

    // Construct matrix N: 2n*11
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(2 * n, 11);

    // Fill odd rows (1:2:end in MATLAB is 0,2,4... in C++)
    for (int i = 0; i < n; ++i) {
        N.row(2*i) << u1[i], 
                    u1[i]*zm[i] - x[i],
                    2*u1[i]*ym[i],
                    -2*z[i] - 2*u1[i]*xm[i],
                    2*y[i],
                    -x[i] - u1[i]*zm[i],
                    -2*y[i],
                    2*u1[i]*xm[i] - 2*z[i],
                    x[i] - u1[i]*zm[i],
                    2*u1[i]*ym[i],
                    x[i] + u1[i]*zm[i];
    }

    // Fill even rows (2:2:end in MATLAB is 1,3,5... in C++)
    for (int i = 0; i < n; ++i) {
        N.row(2*i + 1) << v1[i],
                        v1[i]*zm[i] - y[i],
                        2*z[i] + 2*v1[i]*ym[i],
                        -2*v1[i]*xm[i],
                        -2*x[i],
                        y[i] - v1[i]*zm[i],
                        -2*x[i],
                        2*v1[i]*xm[i],
                        -y[i] - v1[i]*zm[i],
                        2*v1[i]*ym[i] - 2*z[i],
                        y[i] + v1[i]*zm[i];
    }

    // Compute MTN (sum of odd and even rows separately)
    Eigen::MatrixXd MTN(2, 11);
    // Replace seqN with manual row selection
    MTN.row(0).setZero();
    MTN.row(1).setZero();
    for (int i = 0; i < n; i++) {
        if (i % 2 == 0) {
            MTN.row(0) += N.row(i);  // Sum of odd rows
        } else {
            MTN.row(1) += N.row(i);  // Sum of even rows
        }
    }

    // Construct matrix Q: 11*11
    Eigen::MatrixXd Q = N.transpose() * N - (1.0/n) * MTN.transpose() * MTN;

    // Extract const and q
    double const_val = Q(0,0);
    Eigen::VectorXd q = Q.block(0, 1, 1, 10);
    Q = Q.block(1, 1, 10, 10);    

    // Extract elements from Q matrix (using zero-based indexing)
    const double Q11 = Q(0,0), Q12 = Q(0,1), Q13 = Q(0,2), Q14 = Q(0,3), Q15 = Q(0,4), Q16 = Q(0,5), Q17 = Q(0,6), Q18 = Q(0,7), Q19 = Q(0,8), Q110 = Q(0,9);
    const double                 Q22 = Q(1,1), Q23 = Q(1,2), Q24 = Q(1,3), Q25 = Q(1,4), Q26 = Q(1,5), Q27 = Q(1,6), Q28 = Q(1,7), Q29 = Q(1,8), Q210 = Q(1,9);
    const double                             Q33 = Q(2,2), Q34 = Q(2,3), Q35 = Q(2,4), Q36 = Q(2,5), Q37 = Q(2,6), Q38 = Q(2,7), Q39 = Q(2,8), Q310 = Q(2,9);
    const double                                         Q44 = Q(3,3), Q45 = Q(3,4), Q46 = Q(3,5), Q47 = Q(3,6), Q48 = Q(3,7), Q49 = Q(3,8), Q410 = Q(3,9);
    const double                                                     Q55 = Q(4,4), Q56 = Q(4,5), Q57 = Q(4,6), Q58 = Q(4,7), Q59 = Q(4,8), Q510 = Q(4,9);
    const double                                                                 Q66 = Q(5,5), Q67 = Q(5,6), Q68 = Q(5,7), Q69 = Q(5,8), Q610 = Q(5,9);
    const double                                                                             Q77 = Q(6,6), Q78 = Q(6,7), Q79 = Q(6,8), Q710 = Q(6,9);
    const double                                                                                         Q88 = Q(7,7), Q89 = Q(7,8), Q810 = Q(7,9);
    const double                                                                                                     Q99 = Q(8,8), Q910 = Q(8,9);
    const double                                                                                                                 Q1010 = Q(9,9);

    // Extract elements from q vector
    const double q1 = q(0), q2 = q(1), q3 = q(2), q4 = q(3), q5 = q(4), q6 = q(5), q7 = q(6), q8 = q(7), q9 = q(8), q10 = q(9);

    // Initialize coefficient vectors
    Eigen::VectorXd coeff1(24), coeff2(24), coeff3(24), coeff4(24);

    // Fill coefficient vectors
    coeff1 << 4*Q11, 6*Q12, 6*Q13, 6*Q14, 4*Q15 + 2*Q22, 4*Q16 + 4*Q23, 4*Q17 + 4*Q24, 
            4*Q18 + 2*Q33, 4*Q19 + 4*Q34, 4*Q110 + 2*Q44, 4*q1, 2*Q25, 2*Q26 + 2*Q35, 
            2*Q27 + 2*Q45, 2*Q28 + 2*Q36, 2*Q29 + 2*Q37 + 2*Q46, 2*Q210 + 2*Q47, 2*q2, 
            2*Q38, 2*Q39 + 2*Q48, 2*Q310 + 2*Q49, 2*q3, 2*Q410, 2*q4;

    coeff2 << 2*Q12, 4*Q15 + 2*Q22, 2*Q16 + 2*Q23, 2*Q17 + 2*Q24, 6*Q25, 4*Q26 + 4*Q35, 
            4*Q27 + 4*Q45, 2*Q28 + 2*Q36, 2*Q29 + 2*Q37 + 2*Q46, 2*Q210 + 2*Q47, 2*q2, 
            4*Q55, 6*Q56, 6*Q57, 4*Q58 + 2*Q66, 4*Q59 + 4*Q67, 4*Q510 + 2*Q77, 4*q5, 
            2*Q68, 2*Q69 + 2*Q78, 2*Q610 + 2*Q79, 2*q6, 2*Q710, 2*q7;

    coeff3 << 2*Q13, 2*Q16 + 2*Q23, 4*Q18 + 2*Q33, 2*Q19 + 2*Q34, 2*Q26 + 2*Q35, 
            4*Q28 + 4*Q36, 2*Q29 + 2*Q37 + 2*Q46, 6*Q38, 4*Q39 + 4*Q48, 2*Q310 + 2*Q49, 
            2*q3, 2*Q56, 4*Q58 + 2*Q66, 2*Q59 + 2*Q67, 6*Q68, 4*Q69 + 4*Q78, 
            2*Q610 + 2*Q79, 2*q6, 4*Q88, 6*Q89, 4*Q810 + 2*Q99, 4*q8, 2*Q910, 2*q9;

    coeff4 << 2*Q14, 2*Q17 + 2*Q24, 2*Q19 + 2*Q34, 4*Q110 + 2*Q44, 2*Q27 + 2*Q45, 
            2*Q29 + 2*Q37 + 2*Q46, 4*Q210 + 4*Q47, 2*Q39 + 2*Q48, 4*Q310 + 4*Q49, 
            6*Q410, 2*q4, 2*Q57, 2*Q59 + 2*Q67, 4*Q510 + 2*Q77, 2*Q69 + 2*Q78, 
            4*Q610 + 4*Q79, 6*Q710, 2*q7, 2*Q89, 4*Q810 + 2*Q99, 6*Q910, 2*q9, 
            4*Q1010, 4*q10;

    // Normalize coefficients
    coeff1.normalize();  // Equivalent to coeff1 = coeff1/norm(coeff1)
    coeff2.normalize();
    coeff3.normalize();
    coeff4.normalize();

    // Solve using Grobner basis method
    std::vector<double> xx, yy, zz, tt;
    if (!solveGrobnerBasis(coeff1, coeff2, coeff3, coeff4, xx, yy, zz, tt)) {
        return false;
    }

    // Filter repetitive solutions
    filterRepetitiveSolutions(xx, yy, zz, tt);

    // Polish solutions
    for (size_t i = 0; i < xx.size(); ++i) {
        Eigen::Vector4d solution(xx[i], yy[i], zz[i], tt[i]);
        double obj_cur;  // Current objective value
        bool label_psd;  // PSD label from polish function
        
        if (polishSolution(Q, q, solution, label_psd, obj_cur)) {
            xx[i] = solution(0);
            yy[i] = solution(1);
            zz[i] = solution(2);
            tt[i] = solution(3);
        }
    }

    // Containers for solutions
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> translations;
    std::vector<double> errors;

    for (size_t i = 0; i < xx.size(); ++i) {
        const double a = xx[i], b = yy[i], c = zz[i], d = tt[i];
        
        // Check lambda condition
        const double lambda1 = 1.0/(a*a + b*b + c*c + d*d);
        if (lambda1 > 1e10) {
            continue;
        }
        
        // Construct vec for translation computation
        Eigen::VectorXd vec(11);
        vec << 1, a*a, a*b, a*c, a*d, b*b, b*c, b*d, c*c, c*d, d*d;
        
        // Compute translation
        Eigen::Vector3d t = (1.0/n) * MTN * vec;
        
        // Scale quaternion parameters
        const double scale = std::sqrt(lambda1);
        const double as = a*scale, bs = b*scale, cs = c*scale, ds = d*scale;
        
        // Construct rotation matrix
        Eigen::Matrix3d R;
        R << as*as + bs*bs - cs*cs - ds*ds, 2*(bs*cs - as*ds),           2*(bs*ds + as*cs),
            2*(bs*cs + as*ds),           as*as - bs*bs + cs*cs - ds*ds, 2*(cs*ds - as*bs),
            2*(bs*ds - as*cs),           2*(cs*ds + as*bs),           as*as - bs*bs - cs*cs + ds*ds;
        
        // Update translation
        t = Eigen::Vector3d(lambda1*t(0),
                        lambda1*t(1),
                        lambda1 - R.row(2)*Ucent);
        
        // Check projection condition
        Eigen::MatrixXd proj = R * points3D_mat + t * Eigen::RowVectorXd::Ones(n);
        if (proj.row(2).minCoeff() < 0) {
            continue;
        }
        
        // Compute error
        double err = vec.transpose() * 
                    Eigen::MatrixXd::Zero(11, 11)  // Need to construct full matrix from [const q;q.' Q]
                    * vec;
        
        // Store solution
        rotations.push_back(R);
        translations.push_back(t);
        errors.push_back(err);
    }

    // Check if we found any solutions
    if (errors.empty()) {
        return false;
    }

    // Find solution with minimum error
    size_t best_idx = std::min_element(errors.begin(), errors.end()) - errors.begin();

    // Set the projection matrix [R|t]
    proj_matrix->leftCols<3>() = rotations[best_idx];
    proj_matrix->rightCols<1>() = translations[best_idx];

    return true;
}

bool OPnPEstimator::solveGrobnerBasis(
    const Eigen::VectorXd& coeff1,
    const Eigen::VectorXd& coeff2,
    const Eigen::VectorXd& coeff3,
    const Eigen::VectorXd& coeff4,
    std::vector<double>& x,
    std::vector<double>& y,
    std::vector<double>& z,
    std::vector<double>& t) {

    // Setup solver settings (just like MATLAB)
    SolverSettings settings;
    settings.template_data = OPnPEstimator::template_data_;;  

    // Stack coefficients into matrix (4 x n)
    Eigen::Matrix<double, 4, Eigen::Dynamic> coefficients;
    coefficients.resize(4, coeff1.size());
    coefficients.row(0) = coeff1;
    coefficients.row(1) = coeff2;
    coefficients.row(2) = coeff3;
    coefficients.row(3) = coeff4;

    // Call solver
    Eigen::MatrixXd solutions, stats;
    if (!solverPFold(coefficients, settings, solutions, stats)) {
        return false;
    }

    // Extract solutions (just like MATLAB)
    const int num_solutions = solutions.cols();
    x.resize(num_solutions);
    y.resize(num_solutions);
    z.resize(num_solutions);
    t.resize(num_solutions);

    for (int i = 0; i < num_solutions; ++i) {
        x[i] = solutions(0, i);
        y[i] = solutions(1, i);
        z[i] = solutions(2, i);
        t[i] = solutions(3, i);
    }

    return true;
}

void OPnPEstimator::filterRepetitiveSolutions(
        std::vector<double>& x,
        std::vector<double>& y,
        std::vector<double>& z,
        std::vector<double>& t) {
    
    const double threshold = 1e-4;
    
    for (size_t i = 0; i < x.size(); ) {
        std::vector<size_t> indices_to_remove;
        
        // Compare with all subsequent solutions
        for (size_t j = i + 1; j < x.size(); ++j) {
            // Create vectors for comparison
            Eigen::Vector4d sol_i(x[i], y[i], z[i], t[i]);
            Eigen::Vector4d sol_j(x[j], y[j], z[j], t[j]);
            
            // Check if solutions are similar or negatives of each other
            if ((sol_i - sol_j).norm() < threshold || 
                (sol_i + sol_j).norm() < threshold) {
                indices_to_remove.push_back(j);
            }
        }
        
        // Remove solutions in reverse order to maintain valid indices
        for (auto it = indices_to_remove.rbegin(); it != indices_to_remove.rend(); ++it) {
            x.erase(x.begin() + *it);
            y.erase(y.begin() + *it);
            z.erase(z.begin() + *it);
            t.erase(t.begin() + *it);
        }
        
        // Only increment i if we didn't remove the current solution
        if (indices_to_remove.empty()) {
            ++i;
        }
    }
}

bool OPnPEstimator::solverPFold(
    const Eigen::MatrixXd& C0_in,
    const SolverSettings& settings,
    Eigen::MatrixXd& solutions,
    Eigen::MatrixXd& stats) {

    // Use template data from settings
    TemplateData tmpl = settings.template;

    // Reorder coefficients according to template
    Eigen::MatrixXd C0 = C0_in;
    for (int i = 0; i < C0.cols(); ++i) {
        C0.col(i) = C0_in.col(tmpl.reorder[i] - 1);  // -1 for 0-based indexing
    }

    // Normalize coefficients
    for (int i = 0; i < C0.rows(); ++i) {
        C0.row(i).normalize();
    }

    const int neq = C0.rows();    // number of equations
    const int nmon = tmpl.mon;   // number of monomials

    // Create sparse elimination template
    Eigen::SparseMatrix<double> C(tmpl.rows, tmpl.cols);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(tmpl.II.size());

    // Fill sparse matrix using II, JJ indices from template
    const Eigen::MatrixXd C1 = C0.transpose();
    for (size_t k = 0; k < tmpl.II.size(); ++k) {
        int i = tmpl.II[k] - 1;  // Convert to 0-based indexing
        int j = tmpl.JJ[k] - 1;
        int t_idx = tmpl.T[k] - 1;
        if (t_idx < C1.size()) {  // Bounds check
            triplets.emplace_back(i, j, C1(t_idx));
        }
    }
    C.setFromTriplets(triplets.begin(), triplets.end());

    // Update stats
    stats.resize(4, 1);  // Make room for stats
    stats(0, 0) = C.rows();  // neq
    stats(1, 0) = C.cols();  // nmon


    const int ne = tmpl.E.size();  // number of excessive monomials
    const int nr = tmpl.R.size();  // number of reducible monomials
    const int np = tmpl.P.size();  // number of permissible monomials

    // Debug statistics if needed
    if (settings.debug) {
        stats(2, 0) = nr;  // n_to_reduce
        stats(3, 0) = ne;  // n_excessive
    }

    // Create V = [E R P]
    std::vector<int> V;
    V.reserve(ne + nr + np);
    V.insert(V.end(), tmpl.E.begin(), tmpl.E.end());
    V.insert(V.end(), tmpl.R.begin(), tmpl.R.end());
    V.insert(V.end(), tmpl.P.begin(), tmpl.P.end());

    // Reorder C columns: C = C(:, [E, R, P])
    Eigen::SparseMatrix<double> C_reordered(C.rows(), C.cols());
    std::vector<Eigen::Triplet<double>> reorder_triplets;
    for (int j = 0; j < V.size(); ++j) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(C, V[j]-1); it; ++it) {
            reorder_triplets.emplace_back(it.row(), j, it.value());
        }
    }

    C_reordered.setFromTriplets(reorder_triplets.begin(), reorder_triplets.end());
    C = C_reordered;

    // QR decomposition of excessive monomials part
    Eigen::MatrixXd C2;
    int k = 0;

    if (!tmpl.E.empty()) {
        // [qq rr ee] = qr(C(:, 1 : length(E)))
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(C.leftCols(ne));  // Eigen will convert to dense automatically
        Eigen::MatrixXd qq = qr.householderQ();
        Eigen::MatrixXd rr = qr.matrixQR().triangularView<Eigen::Upper>();
        
        // C2 = [rr qq'*C(:, length(E) + 1 : end)]
        C2.resize(C.rows(), C.cols());
        C2 << rr, qq.transpose() * C.rightCols(C.cols() - ne);
        
        // kk = abs(rr(1))./abs(diag(rr)) < settings.cutoff_threshold
        double first_diag = std::abs(rr(0,0));
        std::vector<bool> kk(rr.rows());
        for (int i = 0; i < rr.rows(); ++i) {
            kk[i] = std::abs(rr(0,0))/std::abs(rr(i,i)) < settings.cutoff_threshold;
        }
        
        // k = find(diff(kk) == -1)
        k = rr.rows();  // default if no transition found
        for (int i = 0; i < kk.size()-1; ++i) {
            if (kk[i] && !kk[i+1]) {
                k = i + 1;
                break;
            }
        }
    } else {
        C2 = C;  // Eigen will handle the conversion as needed
        k = 0;
    }

    // Partition C into R- and P-parts
    Eigen::MatrixXd CR = C2.block(k + 1, ne, C2.rows() - (k + 1), nr);  // k+1:end, ne+1:ne+nr
    Eigen::MatrixXd CP = C2.block(k + 1, C2.cols() - np, C2.rows() - (k + 1), np);  // k+1:end, end-np+1:end

    const int mm = CR.rows();    // size(CR, 1)
    const int nn = CR.cols() + CP.cols();  // size(CR, 2) + size(CP, 2)

    // Check if we have enough equations
    if (nn - mm > tmpl.dim) {  
        return false;  // C++ equivalent of MATLAB's error()
    }

    // Eliminate R-monomials using QR (more stable than LU)
    Eigen::HouseholderQR<Eigen::MatrixXd> qr2(CR);
    Eigen::MatrixXd q2 = qr2.householderQ();
    Eigen::MatrixXd UR2_0 = qr2.matrixQR().triangularView<Eigen::Upper>();
    CP = q2.transpose() * CP;

    // Select basis using QR with column pivoting
    Eigen::MatrixXd CP2 = CP.topRows(nr);
    Eigen::MatrixXd CP3 = CP.bottomRows(CP.rows() - nr);

    // QR with column pivoting (ColPivHouseholderQR in Eigen)
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr3(CP3);
    Eigen::MatrixXd r3 = qr3.matrixQR().triangularView<Eigen::Upper>();
    Eigen::PermutationMatrix<Eigen::Dynamic> e = qr3.colsPermutation();

    // Note: r is tmpl.dim from our template
    const int r = tmpl.dim;

    // Apply column permutation and partition matrices
    Eigen::MatrixXd CP4 = CP2.cols(e.indices().head(np - r));  // e(1 : end - r)
    Eigen::MatrixXd CB1 = CP2.cols(e.indices().tail(r));       // e(end - r + 1 : end)
    Eigen::MatrixXd UP3 = r3.topLeftCorner(np - r, np - r);    // r3(1 : np - r, 1 : np - r)
    Eigen::MatrixXd CB2 = r3.topRightCorner(np - r, r);        // r3(1 : np - r, end - r + 1 : end)

    // Create combined permutation vector (1:ne+nr, e+ne+nr)
    std::vector<int> ee;
    ee.reserve(ne + nr + np);
    for(int i = 0; i < ne + nr; ++i) {
        ee.push_back(i);
    }
    Eigen::VectorXi e_indices = e.indices().array() + (ne + nr);
    ee.insert(ee.end(), e_indices.data(), e_indices.data() + e_indices.size());

    // Reorder V using ee permutation
    // V = V(ee)
    std::vector<int> V_reordered;
    V_reordered.reserve(ee.size());
    for (const auto& idx : ee) {
        V_reordered.push_back(V[idx]);
    }
    V = V_reordered;

    // Elimination step
    // Celim = [UR2_0(1 : nr + np - r, :) [CP4; UP3]]
    Eigen::MatrixXd Celim(nr + np - r, nr + np - r);
    Celim.leftCols(nr) = UR2_0.topRows(nr + np - r);
    Celim.rightCols(np - r).topRows(nr) = CP4;
    Celim.rightCols(np - r).bottomRows(np - r) = UP3;

    // Solve system: T = -Celim \ [CB1; CB2]
    Eigen::MatrixXd rhs(nr + np - r, r);
    rhs.topRows(nr) = CB1;
    rhs.bottomRows(np - r) = CB2;
    Eigen::MatrixXd T = -Celim.colPivHouseholderQr().solve(rhs);

    // Construct modulo matrix
    // modM = zeros(r, n)
    const int n = nmon;  // total number of monomials
    Eigen::MatrixXd modM = Eigen::MatrixXd::Zero(r, n);

    // modM(:, end - r + 1 : end) = eye(r)
    modM.rightCols(r).setIdentity();

    // modM(:, ne + 1 : end - r) = T'
    modM.block(0, ne + 1, r, n - r - ne) = T.transpose();

    // e = V
    e = V_reordered;  // We already have V_reordered from previous block

    // First part: Initialize and fill ind2
    Eigen::VectorXi ind2 = Eigen::VectorXi::Zero(np);
    for (size_t i = 0; i < tmpl.P.size(); ++i) {
        ind2[tmpl.P[i]] = tmpl.ind[i];
    }

    // Get indices for last r elements: ind2(e(n-r+1:n))
    Eigen::VectorXi last_r_indices(r);
    for (int i = 0; i < r; ++i) {
        last_r_indices[i] = ind2[e[n - r + i]];
    }

    // Helper function find_id implementation
    auto find_id = [](const std::vector<int>& list, const Eigen::VectorXi& ids) {
        std::vector<int> ids_reorder(list.size());
        
        // Create lookup table (ccc in MATLAB)
        std::vector<int> lookup(list.size(), -1);  // -1 as sentinel
        for (int i = 0; i < ids.size(); ++i) {
            if (ids[i] < lookup.size()) {
                lookup[ids[i]] = i;
            }
        }
        
        // Fill reordered indices
        for (size_t i = 0; i < list.size(); ++i) {
            if (list[i] < lookup.size()) {
                ids_reorder[i] = lookup[list[i]];
            }
        }
        
        return ids_reorder;
    };

    // Call find_id
    std::vector<int> action_var_indices = find_id(e, last_r_indices);

    // Create action matrix M
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, r);
    for (int i = 0; i < r; ++i) {
        if (action_var_indices[i] >= 0 && action_var_indices[i] < n) {
            M.row(action_var_indices[i]).setZero();
            M(action_var_indices[i], i) = 1.0;
        }
    }

    // Compute final action matrix
    Eigen::MatrixXd m = modM * M;

    // Compute eigenvalues and eigenvectors
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(m.transpose());
    Eigen::VectorXcd d = eigensolver.eigenvalues();
    Eigen::MatrixXcd vv = eigensolver.eigenvectors();

    // Find non-zero rows in modM (okmon)
    std::vector<int> okmon;
    for (int j = 0; j < modM.cols(); ++j) {
        if (modM.col(j).sum() != 0) {
            okmon.push_back(j);
        }
    }

    // Update eigenvectors
    Eigen::MatrixXd modM_reduced(modM.rows(), okmon.size());
    for (size_t i = 0; i < okmon.size(); ++i) {
        modM_reduced.col(i) = modM.col(okmon[i]);
    }
    vv = modM_reduced.transpose() * vv;

    if (settings.p_inverter != "all" && settings.p_inverter != "best") {
        // Get specific inverter id
        int sid = std::stoi(settings.p_inverter);  // Assuming p_inverter is string of number
        
        // Create range 1:p excluding sid
        const int p = tmpl.P.size();  // or wherever p is defined
        std::vector<int> sid_r;
        for (int i = 0; i < p; ++i) {
            if (i + 1 != sid) {  // +1 because MATLAB is 1-based
                sid_r.push_back(i + 1);
            }
        }
        
        // Find indices for a1 and a3
        std::vector<int> a1_a3_indices = {tmpl.ids_a1[sid-1], tmpl.ids_a3[sid-1]};  // -1 for 0-based
        std::vector<int> ids_a13 = find_id(e, a1_a3_indices);
        
        // Adjust indices by subtracting ne
        for (auto& idx : ids_a13) {
            idx -= ne;
        }
        
        // Combine indices
        std::vector<int> ids;
        ids.insert(ids.end(), ids_a13.begin(), ids_a13.end());
        
        // Add abc indices
        ids.push_back(tmpl.ids_abc[sid-1]);
        for (int sr : sid_r) {
            ids.push_back(tmpl.ids_abc[sr-1]);
        }
        
        // Compute ratios and constants
        Eigen::VectorXcd vv1 = (vv.row(ids[1]).array() / vv.row(ids[0]).array()).sqrt();
        Eigen::VectorXcd constant = vv.row(ids[0]).array() / vv1.array();
        
        // Handle real/complex solutions
        std::vector<int> realid;
        if (settings.real_solutions_only) {
            // Find indices where imaginary part is zero (within tolerance)
            const double imag_tol = 1e-10;
            for (int i = 0; i < vv1.size(); ++i) {
                if (std::abs(vv1[i].imag()) < imag_tol) {
                    realid.push_back(i);
                }
            }
            
            // Filter vv1 and constant to keep only real solutions
            Eigen::VectorXcd vv1_filtered(realid.size());
            Eigen::VectorXcd constant_filtered(realid.size());
            for (size_t i = 0; i < realid.size(); ++i) {
                vv1_filtered[i] = vv1[realid[i]];
                constant_filtered[i] = constant[realid[i]];
            }
            vv1 = vv1_filtered;
            constant = constant_filtered;
        } else {
            // Keep all solutions
            realid.resize(tmpl.dim);
            std::iota(realid.begin(), realid.end(), 0);  // Fill with 0,1,2,...,dim-1
        }
        
        // Compute negative solutions
        Eigen::VectorXcd vv1_neg = -vv1;
    
        // Store inverter id
        int mmid = sid;
    }

    else {
        // Find indices for a1 and a3 combined
        std::vector<int> combined_a1_a3;
        combined_a1_a3.insert(combined_a1_a3.end(), tmpl.ids_a1.begin(), tmpl.ids_a1.end());
        combined_a1_a3.insert(combined_a1_a3.end(), tmpl.ids_a3.begin(), tmpl.ids_a3.end());
        std::vector<int> ids_a13_list = find_id(e, combined_a1_a3);
        
        // Adjust indices by subtracting ne
        for (auto& idx : ids_a13_list) {
            idx -= ne;
        }
        
        // Get first 4 elements of ids_a13_list
        std::vector<int> ids_a1(ids_a13_list.begin(), ids_a13_list.begin() + 4);
        
        // Initialize solutions matrix
        Eigen::MatrixXd sols;  // Empty matrix for now
        
        // Create containers for cell arrays
        std::vector<std::vector<int>> sid_rl(p);         // For sid_rl{ii}
        std::vector<std::vector<int>> idsl(p);           // For idsl{ii}
        std::vector<Eigen::VectorXcd> vv1l(p);           // For vv1l{ii}
        std::vector<Eigen::VectorXcd> vv1rl(p);          // For vv1rl{ii}
        std::vector<Eigen::VectorXcd> constant(p);        // For constant{ii}
        std::vector<bool> fail_flag(p);                  // For fail_flag
        
        // Main loop
        for (int ii = 0; ii < p; ++ii) {  // Note: 0-based indexing in C++
            // Get pairs of indices from ids_a13_list
            std::vector<int> ids_a13 = {ids_a13_list[ii], ids_a13_list[p + ii]};
            
            // Create range 1:p excluding ii+1
            sid_rl[ii].reserve(p-1);
            for (int j = 0; j < p; ++j) {
                if (j != ii) {
                    sid_rl[ii].push_back(j);
                }
            }
            
            // Store indices
            idsl[ii] = ids_a13;
            
            // Compute sqrt(a3/a1) ratio
            vv1l[ii] = (vv.row(ids_a13[1]).array() / vv.row(ids_a13[0]).array()).sqrt();
            
            // Find real solutions
            std::vector<int> realid;
            const double imag_tol = 1e-10;
            for (int j = 0; j < vv1l[ii].size(); ++j) {
                if (std::abs(vv1l[ii][j].imag()) < imag_tol) {
                    realid.push_back(j);
                }
            }
            
            // Store real solutions
            vv1rl[ii] = Eigen::VectorXcd(realid.size());
            for (size_t j = 0; j < realid.size(); ++j) {
                vv1rl[ii][j] = vv1l[ii][realid[j]];
            }
            
            // Compute constants
            constant[ii] = vv.row(ids_a13[0]).array() / vv1l[ii].array();
            
            // Check for failure
            fail_flag[ii] = vv1rl[ii].size() == 0;

            // init a new variable instead of mm
            std::vector<double> min_norms(p, std::numeric_limits<double>::infinity());
            std::vector<double> mx(p, 1.0);

            if (!fail_flag[ii]) {
                double min_abs_real = std::numeric_limits<double>::infinity();
                for (int j = 0; j < vv1l[ii].size(); ++j) {
                    min_abs_real = std::min(min_abs_real, std::abs(vv1l[ii][j].real()));
                }
                
                if (min_abs_real > 1e-7) {
                    min_norms[ii] = std::abs(vv1l[ii]).minCoeff();
                    mx[ii] = std::abs(vv1rl[ii]).maxCoeff();
                }
            }

            if (settings.p_inverter == "all") {
                // Extract rows from vv using ids_a1[sid_rl[ii]]
                Eigen::MatrixXcd vv2l_ii(3, realid.size());
                for (size_t r = 0; r < sid_rl[ii].size(); r++) {
                    vv2l_ii.row(r) = vv.row(ids_a1[sid_rl[ii][r]]).rightCols(realid.size());
                }
                
                // Element-wise division with constant
                vv2l_ii = vv2l_ii.array() / (Eigen::MatrixXcd::Ones(3,1) * constant[ii].transpose()).array();
                
                // Construct solutions matrix
                Eigen::MatrixXcd solsl(4, 2 * vv1rl[ii].size());
                solsl.topRows(1) << vv1rl[ii].transpose(), -vv1rl[ii].transpose();
                solsl.bottomRows(3) << vv2l_ii, -vv2l_ii;
                
                // Append to sols
                if (sols.size() == 0) {
                    sols = solsl;
                } else {
                    Eigen::MatrixXcd new_sols(sols.rows(), sols.cols() + solsl.cols());
                    new_sols << sols, solsl;
                    sols = new_sols;
                }
            }        
        }
    }

    if (settings.p_inverter != "all") {
        // Compute vv2 using indices from the best mmid
        Eigen::MatrixXcd vv2(3, realid.size());
        for (size_t r = 0; r < sid_rl[mmid].size(); r++) {
            vv2.row(r) = vv.row(ids_a1[sid_rl[mmid][r]]);
        }
        vv2 = vv2.array() / (Eigen::MatrixXcd::Ones(3,1) * constant.transpose()).array();
        
        // Compute negative solutions
        Eigen::MatrixXcd vv2_neg = -vv2;
        
        // Construct full solutions matrix
        Eigen::MatrixXcd sols(4, 2 * vv1.size());
        sols.topRows(1) << vv1.transpose(), vv1_neg.transpose();
        sols.bottomRows(3) << vv2, vv2_neg;
        
        // Reorder rows using [sid,sid_r]
        Eigen::MatrixXcd sols_reordered = sols;
        std::vector<int> reorder_indices;
        reorder_indices.push_back(sid);
        reorder_indices.insert(reorder_indices.end(), sid_r.begin(), sid_r.end());
        for (size_t i = 0; i < reorder_indices.size(); i++) {
            sols.row(i) = sols_reordered.row(reorder_indices[i]);
        }
    }

    if (settings.real_solutions_only) {
        // Find columns where all imaginary parts are < 1e-4
        std::vector<int> realid;
        for (int col = 0; col < sols.cols(); ++col) {
            bool is_real = true;
            for (int row = 0; row < 4; ++row) {  // Check all 4 rows
                if (std::abs(sols(row, col).imag()) >= 1e-4) {
                    is_real = false;
                    break;
                }
            }
            if (is_real) {
                realid.push_back(col);
            }
        }

        // Keep only real solutions
        if (!realid.empty()) {
            Eigen::MatrixXcd sols_real(sols.rows(), realid.size());
            for (size_t i = 0; i < realid.size(); ++i) {
                sols_real.col(i) = sols.col(realid[i]);
            }
            sols = sols_real;
        } else {
            sols.resize(sols.rows(), 0);  // Empty matrix if no real solutions
        }
    }


    return true;
}

void OPnPEstimator::loadTemplateData(TemplateData& tmpl) {
    std::ifstream file("template_pnp.csv");
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open template_pnp.csv");
    }

    // Skip header line
    std::string line;
    std::getline(file, line);

    // Read data line
    std::getline(file, line);
    std::stringstream ss(line);
    std::string token;

    // Helper function to parse array from string
    auto parseArray = [](const std::string& str, auto& vec) {
        std::stringstream ss(str);
        std::string num;
        while (std::getline(ss, num, ',')) {
            // Remove non-numeric characters
            num.erase(std::remove_if(num.begin(), num.end(), 
                [](char c) { return !std::isdigit(c); }), num.end());
            if (!num.empty()) {
                vec.push_back(std::stoi(num));
            }
        }
    };

    // Parse each column
    while (std::getline(ss, token, ',')) {
        if (token.find("dim") != std::string::npos) {
            tmpl.dim = std::stoi(token);
        } else if (token.find("basis_size") != std::string::npos) {
            tmpl.basis_size = std::stoi(token);
        } else if (token.find("P") != std::string::npos) {
            parseArray(token, tmpl.P);
        } else if (token.find("ind") != std::string::npos) {
            parseArray(token, tmpl.ind);
        } else if (token.find("R") != std::string::npos) {
            parseArray(token, tmpl.R);
        } else if (token.find("E") != std::string::npos) {
            parseArray(token, tmpl.E);
        } else if (token.find("ids_a1") != std::string::npos) {
            parseArray(token, tmpl.ids_a1);
        } else if (token.find("ids_a3") != std::string::npos) {
            parseArray(token, tmpl.ids_a3);
        } else if (token.find("ids_abc") != std::string::npos) {
            parseArray(token, tmpl.ids_abc);
        } else if (token.find("ids_ac2") != std::string::npos) {
            parseArray(token, tmpl.ids_ac2);
        } else if (token.find("ids_a2c") != std::string::npos) {
            parseArray(token, tmpl.ids_a2c);
        } else if (token.find("I") != std::string::npos) {
            parseArray(token, tmpl.I);
        } else if (token.find("J") != std::string::npos) {
            parseArray(token, tmpl.J);
        } else if (token.find("II") != std::string::npos) {
            parseArray(token, tmpl.II);
        } else if (token.find("JJ") != std::string::npos) {
            parseArray(token, tmpl.JJ);
        } else if (token.find("reorder") != std::string::npos) {
            parseArray(token, tmpl.reorder);
        } else if (token.find("mon") != std::string::npos) {
            // Parse 2D array for mon
            // This needs special handling as it's a vector of vectors
            std::vector<std::vector<uint8_t>> mon;
            // Implementation for parsing mon...
        } else if (token.find("p_inverter") != std::string::npos) {
            tmpl.p_inverter = "all";  // Default value
        } else if (token.find("T") != std::string::npos) {
            parseArray(token, tmpl.T);
        } else if (token.find("Tid") != std::string::npos) {
            parseArray(token, tmpl.Tid);
        }
    }
}

static void OPnPEstimator::getTemplateData() {
    loadTemplateData(template_data_);
}

bool OPnPEstimator::polishSolution(const Eigen::MatrixXd& Q, 
                   const Eigen::VectorXd& q,
                   Eigen::Vector4d& solution,
                   bool& label_psd,
                   double& obj_cur) {
    // Extract elements from Q matrix (using zero-based indexing)
    const double Q11 = Q(0,0), Q12 = Q(0,1), Q13 = Q(0,2), Q14 = Q(0,3), Q15 = Q(0,4), Q16 = Q(0,5), Q17 = Q(0,6), Q18 = Q(0,7), Q19 = Q(0,8), Q110 = Q(0,9);
    const double                 Q22 = Q(1,1), Q23 = Q(1,2), Q24 = Q(1,3), Q25 = Q(1,4), Q26 = Q(1,5), Q27 = Q(1,6), Q28 = Q(1,7), Q29 = Q(1,8), Q210 = Q(1,9);
    const double                             Q33 = Q(2,2), Q34 = Q(2,3), Q35 = Q(2,4), Q36 = Q(2,5), Q37 = Q(2,6), Q38 = Q(2,7), Q39 = Q(2,8), Q310 = Q(2,9);
    const double                                         Q44 = Q(3,3), Q45 = Q(3,4), Q46 = Q(3,5), Q47 = Q(3,6), Q48 = Q(3,7), Q49 = Q(3,8), Q410 = Q(3,9);
    const double                                                     Q55 = Q(4,4), Q56 = Q(4,5), Q57 = Q(4,6), Q58 = Q(4,7), Q59 = Q(4,8), Q510 = Q(4,9);
    const double                                                                 Q66 = Q(5,5), Q67 = Q(5,6), Q68 = Q(5,7), Q69 = Q(5,8), Q610 = Q(5,9);
    const double                                                                             Q77 = Q(6,6), Q78 = Q(6,7), Q79 = Q(6,8), Q710 = Q(6,9);
    const double                                                                                         Q88 = Q(7,7), Q89 = Q(7,8), Q810 = Q(7,9);
    const double                                                                                                     Q99 = Q(8,8), Q910 = Q(8,9);
    const double                                                                                                                 Q1010 = Q(9,9);

    // Extract elements from q vector (using zero-based indexing)
    const double q1 = q(0), q2 = q(1), q3 = q(2), q4 = q(3), q5 = q(4), q6 = q(5), q7 = q(6), q8 = q(7), q9 = q(8), q10 = q(9);

    // Maximum allowed iterations (one-step implementation)
    constexpr int max_iterations = 1;

    // Damping factors
    double lambda = 1e-8;  // Current damping factor
    constexpr double lambda_max = 1e5;  // Maximum lambda
    constexpr double lambda_min = 1e-8;  // Minimum lambda

    // Success flag
    bool success = false;

    // Get current solution (we already have it in 'solution' parameter)
    double a = solution(0), b = solution(1), c = solution(2), d = solution(3);

    // Construct vector [a^2 a*b a*c a*d b^2 b*c b*d c^2 c*d d^2]
    Eigen::VectorXd vec(10);
    vec << a*a, a*b, a*c, a*d, b*b, b*c, b*d, c*c, c*d, d*d;

    // Compute initial objective value (note: w in MATLAB is our q vector)
    double obj_pre = vec.transpose() * Q * vec + 2.0 * q.transpose() * vec;

    // Iteration loop
    int iteration = 1;
    while (iteration <= max_iterations) {
        // Get current solution values
        a = solution(0);
        b = solution(1);
        c = solution(2);
        d = solution(3);

        // Construct vec [a^2 a*b a*c a*d b^2 b*c b*d c^2 c*d d^2 1]
        Eigen::VectorXd vec(11);
        vec << a*a, a*b, a*c, a*d, b*b, b*c, b*d, c*c, c*d, d*d, 1.0;

        // Construct vec3 with all cubic terms
        Eigen::VectorXd vec3(24);
        vec3 << a*a*a, a*a*b, a*a*c, a*a*d,     // a^3, a^2*b, a^2*c, a^2*d
                a*b*b, a*b*c, a*b*d, a*c*c,     // a*b^2, a*b*c, a*b*d, a*c^2
                a*c*d, a*d*d, a,                 // a*c*d, a*d^2, a
                b*b*b, b*b*c, b*b*d,            // b^3, b^2*c, b^2*d
                b*c*c, b*c*d, b*d*d, b,         // b*c^2, b*c*d, b*d^2, b
                c*c*c, c*c*d, c*d*d, c,         // c^3, c^2*d, c*d^2, c
                d*d*d, d;                        // d^3, d

        // Coefficients for gradient computation
        Eigen::VectorXd coeff1(24), coeff2(24), coeff3(24), coeff4(24);

        // First row of coefficients
        coeff1 << 4*Q11, 6*Q12, 6*Q13, 6*Q14, 4*Q15 + 2*Q22, 4*Q16 + 4*Q23, 4*Q17 + 4*Q24, 
                4*Q18 + 2*Q33, 4*Q19 + 4*Q34, 4*Q110 + 2*Q44, 4*q1, 2*Q25, 2*Q26 + 2*Q35,
                2*Q27 + 2*Q45, 2*Q28 + 2*Q36, 2*Q29 + 2*Q37 + 2*Q46, 2*Q210 + 2*Q47, 2*q2,
                2*Q38, 2*Q39 + 2*Q48, 2*Q310 + 2*Q49, 2*q3, 2*Q410, 2*q4;

        // Second row of coefficients
        coeff2 << 2*Q12, 4*Q15 + 2*Q22, 2*Q16 + 2*Q23, 2*Q17 + 2*Q24, 6*Q25, 4*Q26 + 4*Q35,
                4*Q27 + 4*Q45, 2*Q28 + 2*Q36, 2*Q29 + 2*Q37 + 2*Q46, 2*Q210 + 2*Q47, 2*q2,
                4*Q55, 6*Q56, 6*Q57, 4*Q58 + 2*Q66, 4*Q59 + 4*Q67, 4*Q510 + 2*Q77, 4*q5,
                2*Q68, 2*Q69 + 2*Q78, 2*Q610 + 2*Q79, 2*q6, 2*Q710, 2*q7;

        // Third row of coefficients
        coeff3 << 2*Q13, 2*Q16 + 2*Q23, 4*Q18 + 2*Q33, 2*Q19 + 2*Q34, 2*Q26 + 2*Q35,
                4*Q28 + 4*Q36, 2*Q29 + 2*Q37 + 2*Q46, 6*Q38, 4*Q39 + 4*Q48, 2*Q310 + 2*Q49,
                2*q3, 2*Q56, 4*Q58 + 2*Q66, 2*Q59 + 2*Q67, 6*Q68, 4*Q69 + 4*Q78,
                2*Q610 + 2*Q79, 2*q6, 4*Q88, 6*Q89, 4*Q810 + 2*Q99, 4*q8, 2*Q910, 2*q9;

        // Fourth row of coefficients
        coeff4 << 2*Q14, 2*Q17 + 2*Q24, 2*Q19 + 2*Q34, 4*Q110 + 2*Q44, 2*Q27 + 2*Q45,
                2*Q29 + 2*Q37 + 2*Q46, 4*Q210 + 4*Q47, 2*Q39 + 2*Q48, 4*Q310 + 4*Q49,
                6*Q410, 2*q4, 2*Q57, 2*Q59 + 2*Q67, 4*Q510 + 2*Q77, 2*Q69 + 2*Q78,
                4*Q610 + 4*Q79, 6*Q710, 2*q7, 2*Q89, 4*Q810 + 2*Q99, 6*Q910, 2*q9,
                4*Q1010, 4*q10;

        // Compute gradient
        Eigen::Vector4d g;
        g << coeff1.dot(vec3),
            coeff2.dot(vec3),
            coeff3.dot(vec3),
            coeff4.dot(vec3);

        // Initialize coefficient vectors for Hessian
        Eigen::VectorXd h11(11), h12(11), h13(11), h14(11);
        Eigen::VectorXd h22(11), h23(11), h24(11);
        Eigen::VectorXd h33(11), h34(11);
        Eigen::VectorXd h44(11);

        // First row coefficients
        h11 << 12*Q11, 12*Q12, 12*Q13, 12*Q14, 4*Q15 + 2*Q22, 4*Q16 + 4*Q23, 4*Q17 + 4*Q24, 
            4*Q18 + 2*Q33, 4*Q19 + 4*Q34, 4*Q110 + 2*Q44, 4*q1;

        h12 << 6*Q12, 8*Q15 + 4*Q22, 4*Q16 + 4*Q23, 4*Q17 + 4*Q24, 6*Q25, 4*Q26 + 4*Q35,
            4*Q27 + 4*Q45, 2*Q28 + 2*Q36, 2*Q29 + 2*Q37 + 2*Q46, 2*Q210 + 2*Q47, 2*q2;

        h13 << 6*Q13, 4*Q16 + 4*Q23, 8*Q18 + 4*Q33, 4*Q19 + 4*Q34, 2*Q26 + 2*Q35,
            4*Q28 + 4*Q36, 2*Q29 + 2*Q37 + 2*Q46, 6*Q38, 4*Q39 + 4*Q48, 2*Q310 + 2*Q49, 2*q3;

        h14 << 6*Q14, 4*Q17 + 4*Q24, 4*Q19 + 4*Q34, 8*Q110 + 4*Q44, 2*Q27 + 2*Q45,
            2*Q29 + 2*Q37 + 2*Q46, 4*Q210 + 4*Q47, 2*Q39 + 2*Q48, 4*Q310 + 4*Q49, 6*Q410, 2*q4;

        // Second row coefficients
        h22 << 4*Q15 + 2*Q22, 12*Q25, 4*Q26 + 4*Q35, 4*Q27 + 4*Q45, 12*Q55, 12*Q56,
            12*Q57, 4*Q58 + 2*Q66, 4*Q59 + 4*Q67, 4*Q510 + 2*Q77, 4*q5;

        h23 << 2*Q16 + 2*Q23, 4*Q26 + 4*Q35, 4*Q28 + 4*Q36, 2*Q29 + 2*Q37 + 2*Q46,
            6*Q56, 8*Q58 + 4*Q66, 4*Q59 + 4*Q67, 6*Q68, 4*Q69 + 4*Q78, 2*Q610 + 2*Q79, 2*q6;

        h24 << 2*Q17 + 2*Q24, 4*Q27 + 4*Q45, 2*Q29 + 2*Q37 + 2*Q46, 4*Q210 + 4*Q47,
            6*Q57, 4*Q59 + 4*Q67, 8*Q510 + 4*Q77, 2*Q69 + 2*Q78, 4*Q610 + 4*Q79, 6*Q710, 2*q7;

        // Third row coefficients
        h33 << 4*Q18 + 2*Q33, 4*Q28 + 4*Q36, 12*Q38, 4*Q39 + 4*Q48, 4*Q58 + 2*Q66,
            12*Q68, 4*Q69 + 4*Q78, 12*Q88, 12*Q89, 4*Q810 + 2*Q99, 4*q8;

        h34 << 2*Q19 + 2*Q34, 2*Q29 + 2*Q37 + 2*Q46, 4*Q39 + 4*Q48, 4*Q310 + 4*Q49,
            2*Q59 + 2*Q67, 4*Q69 + 4*Q78, 4*Q610 + 4*Q79, 6*Q89, 8*Q810 + 4*Q99, 6*Q910, 2*q9;

        // Fourth row coefficients
        h44 << 4*Q110 + 2*Q44, 4*Q210 + 4*Q47, 4*Q310 + 4*Q49, 12*Q410, 4*Q510 + 2*Q77,
            4*Q610 + 4*Q79, 12*Q710, 4*Q810 + 2*Q99, 12*Q910, 12*Q1010, 4*q10;

        // Construct Hessian matrix using dot products
        Eigen::Matrix4d H;
        H << h11.dot(vec), h12.dot(vec), h13.dot(vec), h14.dot(vec),
            h12.dot(vec), h22.dot(vec), h23.dot(vec), h24.dot(vec),
            h13.dot(vec), h23.dot(vec), h33.dot(vec), h34.dot(vec),
            h14.dot(vec), h24.dot(vec), h34.dot(vec), h44.dot(vec);
        // Check if Hessian is positive definite using eigenvalues
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(H);
        if (eigensolver.eigenvalues().minCoeff() < -1e-3) {
            // Not positive definite - set failure state
            label_psd = false;
            solution.setZero();  // Set [x,y,z,t] to zero
            obj_cur = std::numeric_limits<double>::infinity();
            return false;
        } else {
            label_psd = true;
        }

        // Store current solution
        Eigen::Vector4d q_temp = solution;

        while (lambda < lambda_max) {
            // Solve for increment using damped Newton
            // (H + lambda*I)\g in MATLAB becomes solve() in Eigen
            Eigen::Matrix4d damped_H = H + lambda * Eigen::Matrix4d::Identity();
            Eigen::Vector4d delta = damped_H.ldlt().solve(g);

            // Update parameters (note: no need for transpose since delta is already correct orientation)
            solution = q_temp - delta;
            
            // Extract current values
            a = solution(0);
            b = solution(1);
            c = solution(2);
            d = solution(3);

            // Construct vector for objective computation
            Eigen::VectorXd vec(10);
            vec << a*a, a*b, a*c, a*d, b*b, b*c, b*d, c*c, c*d, d*d;
            
            // Evaluate objective function
            obj_cur = vec.transpose() * Q * vec + 2.0 * q.transpose() * vec;

            // Check convergence
            if (obj_cur >= obj_pre) {
                // Increase damping if objective didn't improve
                lambda *= 10.0;
                continue;
            } else {
                // Decrease damping if objective improved
                obj_pre = obj_cur;
                lambda *= 0.1;
                break;
            }
        }

        // Check if lambda exceeded maximum - restore previous solution
        if (lambda >= lambda_max) {
            solution = q_temp;
            break;
        }
        
        // Clamp lambda to minimum value
        if (lambda <= lambda_min) {
            lambda = lambda_min;
        }
        
        iteration++;
    }
    return true;
}