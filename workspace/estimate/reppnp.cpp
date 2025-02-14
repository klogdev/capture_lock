#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

#include "estimators/utils.h"

#include "estimate/reppnp.h"
#include <cmath>
#include <algorithm>
#include <numeric>


std::vector<REPPnPEstimator::M_t> REPPnPEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
    std::vector<M_t> models;

    if(points2D.size() < kMinNumSamples || points3D.size() < kMinNumSamples) {
        return models;
    }

    REPPnPEstimator reppnp;
    M_t proj_matrix;

    if (!reppnp.ComputeREPPnPPose(points2D, points3D, &proj_matrix)) {
        return std::vector<REPPnPEstimator::M_t>({});
    }

    return std::vector<REPPnPEstimator::M_t>({proj_matrix});
}

void REPPnPEstimator::Residuals(const std::vector<X_t>& points2D,
                                const std::vector<Y_t>& points3D,
                                const M_t& proj_matrix, std::vector<double>* residuals) {
    residuals->clear();
    colmap::ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}

bool REPPnPEstimator::ComputeREPPnPPose(
    const std::vector<Eigen::Vector2d>& points2D,
    const std::vector<Eigen::Vector3d>& points3D,
    Eigen::Matrix3x4d* proj_matrix,
    double min_error) {
    
    const int sol_iter = 1;  // indicates if the initial solution must be optimized
    const int dims = 4;      // kernel dimensions
    
    // Compute M
    Eigen::Matrix<double, 4, 3> control_points_prep;
    Eigen::MatrixXd alphas;
    Eigen::MatrixXd M;
    PrepareData(alphas, control_points_prep, M, points3D, points2D);

    Eigen::MatrixXd M_working = M;
    
    // Robust kernel estimation
    Eigen::MatrixXd Km;
    std::vector<int> inliers;
    int robust_iters;
    RobustKernelEstimation(Km, inliers, robust_iters, M_working, dims, min_error);
    
    // Create working copies before pose computation
    Eigen::MatrixXd Km_working = Km;
    std::vector<int> inliers_working = inliers;
    
    // Compute pose using kernel (needs 3x4 control points)
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    double error;
    Eigen::Matrix<double, 3, 4> control_points = control_points_prep.transpose();
    bool success = ComputePoseFromKernel(R, t, error, control_points, Km_working, dims, sol_iter);
    
    // Form projection matrix [R|t]
    if (success) {
        proj_matrix->block<3,3>(0,0) = R;
        proj_matrix->block<3,1>(0,3) = t;
    }
    
    return success;
}

void REPPnPEstimator::PrepareData(
    Eigen::MatrixXd& alphas,          
    Eigen::Matrix<double, 4, 3>& control_points,
    Eigen::MatrixXd& M,
    const std::vector<Y_t>& points3D,
    const std::vector<X_t>& points2D) {
    
    // Define control points (always, since we can't check for nullptr with references)
    DefineControlPoints(control_points);
    
    Eigen::Matrix<double, 4, 3> control_points_working = control_points;

    // Create a working copy of points3D, similar to MATLAB's Xw = Pts'
    std::vector<Eigen::Vector3d> points3D_working = points3D;

    // Compute alphas
    ComputeAlphas(alphas, points3D_working, control_points_working);

    Eigen::MatrixXd alphas_working = alphas;

    // Convert points2D to matrix form (similar to MATLAB's U = impts)
    Eigen::MatrixXd U(2, points2D.size());
    for (size_t i = 0; i < points2D.size(); ++i) {
        U.col(i) = points2D[i];
    }
    
    // Now flatten to match MATLAB's U(:)
    Eigen::VectorXd U_flat = Eigen::Map<Eigen::VectorXd>(U.data(), U.size());
    
    // Compute M matrix
    ComputeM(M, U_flat, alphas);
}

void REPPnPEstimator::RobustKernelEstimation(
    Eigen::MatrixXd& Km,
    std::vector<int>& inliers,
    int& robust_iters,
    const Eigen::MatrixXd& M,
    int dims,
    double min_error) {
    
    const int m = M.rows();
    const int id = m / 8;  // MATLAB: round(m/8) but since we're using integer division, it's already rounded
    std::vector<int> idx(m);
    std::iota(idx.begin(), idx.end(), 0);  // Fill with 0,1,2,...,m-1
    
    double prev_sv = std::numeric_limits<double>::infinity();
    const bool pairs = true;  // In MATLAB this is always 1
    
    Eigen::MatrixXd resv;  // Store the best eigenvectors
    std::vector<int> residx;  // Store the best inlier indices
    
    // Main iteration loop (max 10 iterations in MATLAB)
    for (int i = 0; i < 10; ++i) {
        // Extract rows according to current indices
        Eigen::MatrixXd N(idx.size(), M.cols());
        for (size_t j = 0; j < idx.size(); ++j) {
            N.row(j) = M.row(idx[j]);
        }
        
        // Compute SVD
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(N.transpose() * N, Eigen::ComputeFullV);
        const Eigen::MatrixXd& v = svd.matrixV();
        
        // Compute errors (always using pairs=true branch from MATLAB)
        Eigen::VectorXd error21(m/2);
        Eigen::VectorXd error22(m/2);
        for (int j = 0; j < m/2; ++j) {
            error21(j) = M.row(2*j) * v.col(v.cols()-1);
            error22(j) = M.row(2*j+1) * v.col(v.cols()-1);
        }
        Eigen::VectorXd error2 = (error21.array().square() + error22.array().square()).sqrt();

        // [sv, tidx] = sort(error2)
        std::vector<size_t> tidx(error2.size());
        std::iota(tidx.begin(), tidx.end(), 0);  // Fill with 0,1,2,...
        std::sort(tidx.begin(), tidx.end(),
                 [&error2](size_t i1, size_t i2) { return error2(i1) < error2(i2); });
        
        Eigen::VectorXd sv(error2.size());
        for (size_t i = 0; i < error2.size(); ++i) {
            sv(i) = error2(tidx[i]);
        }
        
        // Compute median and number of inliers
        double med = sv(m/8);  // Using floor(m/8) from MATLAB
        int ninliers = (sv.array() < std::max(med, min_error)).count();
        
        // Check if we should stop
        if (med >= prev_sv) {
            break;
        }
        else {
            prev_sv = med;
            resv = v;

            residx.clear();
            for (int j = 0; j < ninliers; ++j) {
                residx.push_back(tidx[j]);
            }
        }
        // Update indices for next iteration (matching MATLAB's ordering)
        idx.clear();
        for (int j = 0; j < ninliers; ++j) {
            idx.push_back(2 * tidx[j] - 1);  // odd indices first
            idx.push_back(2 * tidx[j]);      // even indices second
        }
        
        robust_iters = i + 1;  // Store number of iterations
    }
    
    // Extract final kernel matrix
    Km = resv.rightCols(dims);
    
    // Set output inliers
    inliers = residx;
}

void REPPnPEstimator::ComputeM(
    Eigen::MatrixXd& M,
    const Eigen::VectorXd& U,
    const Eigen::MatrixXd& alphas) {    // Renamed to match parameter name
    
    const int n = alphas.rows();  // number of points
    
    // Create the base pattern [1 0 -1; 0 1 -1]
    Eigen::Matrix<double, 2, 3> pattern;
    pattern << 1, 0, -1,
               0, 1, -1;
    
    // Equivalent to MATLAB's kron(Alph,[1 0 -1; 0 1 -1])
    M.resize(2*n, 12);  // 12 = 4 control points * 3 coordinates
    
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < 4; ++j) {  // 4 control points
            M.block<2,3>(2*i, 3*j) = pattern * alphas(i,j);
        }
    }
    
    // Modify columns 3,6,9,12 (0-based: 2,5,8,11)
    // In MATLAB: M(:,[3,6,9,12]) = M(:,[3,6,9,12]) .* (U * ones(1,4))
    Eigen::MatrixXd U_repeated = U.replicate(1, 4);  // Each U value repeated 4 times
    for (int j = 0; j < 4; ++j) {
        M.col(3*j + 2).array() *= U_repeated.col(j).array();
    }
}

void REPPnPEstimator::DefineControlPoints(Eigen::Matrix<double, 4, 3>& control_points) {
    control_points << 1, 0, 0,
                      0, 1, 0,
                      0, 0, 1,
                      0, 0, 0;
}

void REPPnPEstimator::ComputeAlphas(
    Eigen::MatrixXd& alphas,  // Should be n×4 after computation
    const std::vector<Y_t>& points3D,
    const Eigen::Matrix<double, 4, 3>& control_points) {
    
    const int n = points3D.size();
    
    // Create homogeneous control points matrix (4×4)
    Eigen::Matrix4d C;
    C.block<4,3>(0,0) = control_points;
    C.col(3).setOnes();
    
    // Create homogeneous world points matrix (4×n)
    Eigen::MatrixXd X(4, n);
    for (int i = 0; i < n; ++i) {
        X.block<3,1>(0,i) = points3D[i];
    }
    X.row(3).setOnes();
    
    // Compute alphas by solving the linear system
    // Result will be n×4 after transpose
    alphas = (C.inverse() * X).transpose();
}

bool REPPnPEstimator::ComputePoseFromKernel(
    Eigen::Matrix3d& R,
    Eigen::Vector3d& t,
    double& error,
    const Eigen::Matrix<double, 3, 4>& control_points,
    const Eigen::MatrixXd& Km,
    int dims,
    bool sol_iter) {
    
    // Reshape last column of Km to 3×dims matrix (with hard copy)
    Eigen::VectorXd last_col = Km.col(Km.cols()-1);
    Eigen::MatrixXd vK = Eigen::Map<Eigen::MatrixXd>(last_col.data(), 3, dims);
    
    Eigen::Matrix<double, 3, 4> P;     // Original points (fixed size 3x4)
    Eigen::Vector3d mP;                // Mean of points
    Eigen::Matrix<double, 3, 4> cP;    // Centered points (fixed size 3x4)
    double norm;                       // Norm of centered points
    Eigen::Matrix<double, 3, 4> nP;    // Normalized points

    P = control_points;
    mP = P.rowwise().mean();
    cP = P - mP * Eigen::Matrix<double, 1, 4>::Ones();  // Fixed size 1x4 ones vector
    norm = cP.norm();
    nP = cP / norm;
    
    // Handle sign of vK
    Eigen::Matrix<double, 3, Eigen::Dynamic> vK_signed = vK;
    if (vK.row(2).mean() < 0) {
        vK_signed = -vK;
    }
    
    // Initial Procrustes solution
    double b;
    Eigen::Matrix<double, 3, Eigen::Dynamic> mc;
    Eigen::Matrix3d R_init;
    MyProcrustes(P, mP, nP, norm, vK_signed, R_init, b, mc); 
    
    Eigen::Matrix<double, 3, Eigen::Dynamic> solV = b * vK_signed;
    Eigen::Matrix3d solR = R_init;
    Eigen::Matrix<double, 3, Eigen::Dynamic> solmc = mc;
    
    // Iterative optimization if requested
    if (sol_iter) {
        error = std::numeric_limits<double>::infinity();
        const int n_iterations = 10;

        Eigen::Matrix<double, 3, Eigen::Dynamic> solV_iter = b * vK_signed;
        Eigen::Matrix3d solR_iter = R_init;
        Eigen::Matrix<double, 3, Eigen::Dynamic> solmc_iter = mc;
        
        for (int iter = 0; iter < n_iterations; ++iter) {
            // Project previous solution into null space
            Eigen::Matrix<double, 3, Eigen::Dynamic> A = solR_iter * (-solmc_iter + P);
            Eigen::VectorXd abcd = Km.colPivHouseholderQr().solve(Eigen::Map<const Eigen::VectorXd>(A.data(), A.size()));
            Eigen::VectorXd Kmabcd = Km * abcd;  // Store the result first
            Eigen::Matrix<double, 3, Eigen::Dynamic> newV = Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>>(
                Kmabcd.data(), 3, dims);
            
            // Compute Euclidean error
            double newerr = (solR_iter.transpose() * newV + solmc_iter - P).norm();
            
            if (newerr > error && iter > 1) {
                break;
            }
            
            // Update solution
            MyProcrustes(P, mP, nP, norm, newV, solR_iter, b, solmc_iter); 
            solV_iter = b * newV;
            error = newerr;
        }

        solV = solV_iter;
        solR = solR_iter;
        solmc = solmc_iter;
    }
    
    // Final solution
    R = solR;
    Eigen::Vector3d mV = solV.rowwise().mean();
    t = mV - R * mP;
    
    return true;
}

void REPPnPEstimator::MyProcrustes(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& P,     // Original points
    const Eigen::Vector3d& mP,                             // Mean of points
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& nP,    // Normalized points
    const double norm,                                     // Norm of centered points
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& Y,     // Points to align
    Eigen::Matrix3d& R,                                    // Rotation matrix
    double& b,                                             // Scale factor
    Eigen::Matrix<double, 3, Eigen::Dynamic>& mc) {
    
    const int dims = Y.cols();
    
    // Make working copy of Y
    Eigen::MatrixXd Y_working = Y;
    
    // Center Y points
    Eigen::Vector3d mY = Y_working.rowwise().mean();
    Eigen::MatrixXd cY = Y_working - mY * Eigen::RowVectorXd::Ones(dims);
    
    // Normalize centered points
    double ncY = cY.norm();
    Eigen::MatrixXd tcY = cY / ncY;
    
    // Compute SVD
    Eigen::Matrix3d A = nP * tcY.transpose();
    Eigen::Matrix3d A_working = A;  // Make working copy for SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(A_working, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    // Compute rotation ensuring det(R) = 1 (matching MATLAB)
    Eigen::Matrix3d VUt = svd.matrixV() * svd.matrixU().transpose();
    R = svd.matrixV() * 
        Eigen::Vector3d(1, 1, VUt.determinant()).asDiagonal() * 
        svd.matrixU().transpose();
    
    // Compute scale and translation
    b = svd.singularValues().sum() * norm / ncY;
    Eigen::Vector3d c = mP - b * R.transpose() * mY;
    mc = c * Eigen::RowVectorXd::Ones(dims);
}
