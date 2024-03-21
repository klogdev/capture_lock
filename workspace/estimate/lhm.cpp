#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <limits>
#include <iostream>

#include "estimate/lhm.h"
#include "estimate/least_sqr_pnp.h"
#include "estimate/adj_quat.h"
#include "util_/math_helper.h"

#include "base/projection.h"
#include "base/camera.h"
#include "base/pose.h"
#include "estimators/utils.h"

// init the static members for the instance
LHMOptions LHMEstimator::options_ = LHMOptions();
Eigen::Matrix3x4d* LHMEstimator::gt_pose_ = nullptr;
double LHMEstimator::first_estimated_frob = 0;


std::vector<LHMEstimator::M_t> LHMEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
    LHMEstimator lhm;
    M_t proj_matrix;

    if (!lhm.ComputeLHMPose(points2D, points3D, &proj_matrix)) {
        return std::vector<LHMEstimator::M_t>({});
    }

    return std::vector<LHMEstimator::M_t>({proj_matrix});
}

void LHMEstimator::Residuals(const std::vector<X_t>& points2D,
                      const std::vector<Y_t>& points3D,
                      const M_t& proj_matrix, std::vector<double>* residuals) {
    colmap::ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}

bool LHMEstimator::ComputeLHMPose(const std::vector<Eigen::Vector2d>& points2D,
                                  const std::vector<Eigen::Vector3d>& points3D,
                                  Eigen::Matrix3x4d* proj_matrix) {
    int n_points = points2D.size();
    std::vector<Eigen::Matrix3d> V; // initialize a vector of line of sight matrices
    Eigen::Matrix3d sum_Vk; // summation of V in eqn. 20
    sum_Vk.setZero();

    std::vector<Eigen::Vector3d> homogeneous_pts; // for later use in the weak perspective model

    for(const auto& p : points2D) {
        Eigen::Vector3d homogeneousPoint(p[0], p[1], 1.0);
        homogeneous_pts.push_back(homogeneousPoint);
        double mag = 1.0 / homogeneousPoint.squaredNorm();

        // Compute Vk as the outer product of the homogeneous point
        Eigen::Matrix3d Vk = mag * homogeneousPoint * homogeneousPoint.transpose();

        // Add Vk to the sum
        sum_Vk += Vk;

        // Store Vk
        V.push_back(Vk);
    }

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();  // Identity matrix
    double n_inv = 1.0 / n_points;  // Inverse of the number of points

    // Compute Tfact directly from the eqn. 20, the 1/n lead term already be accounted here
    Eigen::Matrix3d Tfact = (I - n_inv * sum_Vk).inverse() * n_inv;

    // Calculate the initial guess of rotation and translation
    Eigen::Matrix3d init_rot;
    Eigen::Vector3d init_trans;

    if(options_.rot_init_est == "horn"){
        bool weak_persp = WeakPerspectiveQuat(points3D, homogeneous_pts,
                                              init_rot, init_trans); 
    }
    else {
        bool weak_persp = WeakPerspectiveDRaMInit2D(points3D, homogeneous_pts,
                                                    init_rot, init_trans);
    }
    
    // refine the init trans after obtained the init rotation
    TransFromRotLHM(points3D, V, Tfact, init_rot, init_trans);

    if(options_.optim_option == "lhm"){
        bool opt_result = IterationLHM(points3D, V, Tfact, init_rot, init_trans);
    }
    else {
        const Eigen::Matrix3d rot_copy = init_rot;
        Eigen::Vector4d quat_ = colmap::RotationMatrixToQuaternion(rot_copy);
        LeastSquareSolver(points2D, points3D, quat_, init_trans, options_.lhm_iter);
        const Eigen::Vector4d quat_copy = quat_;
        init_rot = colmap::QuaternionToRotationMatrix(quat_copy);
    }

    // Compose the extrinsic matrix
    proj_matrix->block<3, 3>(0, 0) = init_rot;
    proj_matrix->col(3) = init_trans;

    return true;
}

bool LHMEstimator::IterationLHM(const std::vector<Eigen::Vector3d>& points3D,
                                const std::vector<Eigen::Matrix3d>& V,
                                const Eigen::Matrix3d& Tfact,
                                Eigen::Matrix3d& init_rot,
                                Eigen::Vector3d& init_trans) {
    int n_points = points3D.size();
    int iter = 0;
    double curr_err = std::numeric_limits<double>::max();
    double old_err = -1.0; // dummy old error to start the while loop

    std::vector<Eigen::Vector3d> temp_rotated; // update 3D points iteratively
    temp_rotated.resize(n_points);

    while(abs((old_err - curr_err)/old_err) > options_.lhm_tolerance && 
               curr_err > options_.lhm_epsilon && iter < options_.lhm_iter) {
        for (int i = 0; i < n_points; ++i) {
            temp_rotated[i] = init_rot * points3D[i] + init_trans;
        }

        old_err = curr_err;
        curr_err = ObjSpaceLHMErr(temp_rotated, V); // eqn. 17

        for (int i = 0; i < n_points; ++i) {
            temp_rotated[i] = V[i] * temp_rotated[i]; // further polish the points by line of sight projection
        }

        bool update_pose = CalcLHMRotTrans(points3D, temp_rotated, V, Tfact,
                                           init_rot, init_trans);
        iter++;
    }

    return true;
}

bool LHMEstimator::CalcLHMRotTrans(const std::vector<Eigen::Vector3d>& points3D0,
                                   const std::vector<Eigen::Vector3d>& points3D1,
                                   const std::vector<Eigen::Matrix3d>& V,
                                   const Eigen::Matrix3d& Tfact,
                                   Eigen::Matrix3d& R,
                                   Eigen::Vector3d& t) {
    if (points3D0.size() < 3) {
        std::cerr << "At least 3 points are necessary for estimating R, t in: " << points3D0.size() << std::endl;
        R.setZero();
        t.setZero();
        return false; // Indicating an error or insufficient data
    }

    Eigen::Vector3d pc = Eigen::Vector3d::Zero();
    Eigen::Vector3d qc = Eigen::Vector3d::Zero();

    // Compute centroids
    for (const auto& p : points3D0) {
        pc += p;
    }
    for (const auto& q : points3D1) {
        qc += q;
    }
    pc /= points3D0.size();
    qc /= points3D1.size();

    // Compute M from centered points
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < points3D0.size(); i++) {
        Eigen::Vector3d p = points3D0[i] - pc;
        Eigen::Vector3d q = points3D1[i] - qc;

        M += q * p.transpose(); // Outer product
    }

    // Perform SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d Vt = svd.matrixV().transpose();

    // Compute Rotation matrix
    R = U * Vt;

    // Computer translation
    TransFromRotLHM(points3D0, V, Tfact, R, t);

    return true;
}

void LHMEstimator::TransFromRotLHM(const std::vector<Eigen::Vector3d>& points3D,
                                   const std::vector<Eigen::Matrix3d>& V,
                                   const Eigen::Matrix3d& Tfact,
                                   const Eigen::Matrix3d& R,
                                   Eigen::Vector3d& t) {
    Eigen::Vector3d sum_term = Eigen::Vector3d::Zero(); // the last sum term of eqn. 20
    
    for (size_t i = 0; i < points3D.size(); i++) {
        //  \sum_(Vk - I) * R * pk
        sum_term += (V[i] - Eigen::Matrix3d::Identity()) * R * points3D[i];
    }

    // t = Tfact * sum_term, here Tfact already averaged by the first 1/n
    t = Tfact * sum_term;
}

double LHMEstimator::ObjSpaceLHMErr(const std::vector<Eigen::Vector3d>& points3D,
                                    const std::vector<Eigen::Matrix3d>& V) {
    double obj_err = 0.0;
    Eigen::Vector3d temp_val;


    for(size_t i = 0; i < points3D.size(); i++){
        temp_val = (Eigen::Matrix3d::Identity() - V[i]) * points3D[i];
        obj_err += temp_val.squaredNorm();
    }

    return obj_err;
}

bool LHMEstimator::WeakPerspectiveQuat(const std::vector<Eigen::Vector3d>& points3D0,
                                       const std::vector<Eigen::Vector3d>& points3D1,
                                       Eigen::Matrix3d& R,
                                       Eigen::Vector3d& t) {
    Eigen::Vector3d pc = Eigen::Vector3d::Zero();
    Eigen::Vector3d qc = Eigen::Vector3d::Zero();

    // here we use (u,v,1) for the points3D1
    GetCentroid(points3D0, points3D1, pc, qc);

    // Compute M from centered points and dq, dp's squares
    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

    double dpsum = 0.0, dqsum = 0.0;

    for (size_t i = 0; i < points3D0.size(); i++) {
        Eigen::Vector3d p = points3D0[i] - pc;
        Eigen::Vector3d q = points3D1[i] - qc;

        dpsum += p.squaredNorm();
        dqsum += q.squaredNorm();

        S += p * q.transpose(); // transform p to q -> p*q.T
    }

    // Diagonal elements
    M(0, 0) = S.trace(); // Sxx + Syy + Szz
    M(1, 1) = S(0, 0) - S(1, 1) - S(2, 2); // Sxx - Syy - Szz
    M(2, 2) = -S(0, 0) + S(1, 1) - S(2, 2); // -Sxx + Syy - Szz
    M(3, 3) = -S(0, 0) - S(1, 1) + S(2, 2); // -Sxx - Syy + Szz

    // Off-diagonal elements
    M(0, 1) = M(1, 0) = S(1, 2) - S(2, 1); // Syz - Szy
    M(0, 2) = M(2, 0) = S(2, 0) - S(0, 2); // Szx - Sxz
    M(0, 3) = M(3, 0) = S(0, 1) - S(1, 0); // Sxy - Syx

    M(1, 2) = M(2, 1) = S(0, 1) + S(1, 0); // Sxy + Syx
    M(1, 3) = M(3, 1) = S(2, 0) + S(0, 2); // Szx + Sxz

    M(2, 3) = M(3, 2) = S(1, 2) + S(2, 1); // Syz + Szy

    double scale = std::sqrt(dqsum / dpsum);

    // Eigenvalue decomposition of a symmetric matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(M);
    if (eigensolver.info() != Eigen::Success) {
        std::cerr << "Eigenvalue decomposition failed!" << std::endl;
        return false;
    }

    // use the eigensolver
    Eigen::Vector4d eigenvec = eigensolver.eigenvectors().col(3); // Last column for largest eigenvalue

    // Convert Quaternion to Rotation Matrix
    // Normalize the eigenvector to ensure it represents a valid rotation
    eigenvec.normalize();

    // Create a Quaternion from the eigenvector
    Eigen::Quaterniond quaternion(eigenvec(0), eigenvec(1), eigenvec(2), eigenvec(3));
    
    // Convert the quaternion to a rotation matrix
    R = quaternion.toRotationMatrix();

    // std::cout << "first estimated matrix is: " << std::endl;
    // std::cout << R << std::endl;

    // Assuming pc and qc are centroids of pts0 and pts1, and scale is computed
    t = qc - scale * (R * pc); // t = qc - R * scale * pc;

    return true;
}

bool LHMEstimator::WeakPerspectiveDRaMInit2D(const std::vector<Eigen::Vector3d>& points3D0,
                                             const std::vector<Eigen::Vector3d>& points3D1,
                                             Eigen::Matrix3d& rot_opt,
                                             Eigen::Vector3d& trans_init) {
    Eigen::Vector3d pc = Eigen::Vector3d::Zero();
    Eigen::Vector3d qc = Eigen::Vector3d::Zero();

    GetCentroid(points3D0, points3D1, pc, qc);
    trans_init = qc - pc; // dummy trans between homogeneaous coord and scene points
    trans_init.z() = 0; // as DRaM not rely on depth, we only translate x, y component

    std::vector<Eigen::Vector3d> shifted_pts;
    std::vector<Eigen::Vector2d> projected_pts;
    for(size_t i = 0; i < points3D0.size(); i++){
        shifted_pts.push_back(points3D0[i] - pc);
        Eigen::Vector3d curr_q = points3D1[i] - qc;
        projected_pts.emplace_back(curr_q.x(), curr_q.y());
    }

    bool bi_correct = BarItzhackOptRot(shifted_pts, projected_pts, rot_opt);
    if (gt_pose_ != nullptr) {
        double matrix_norm = frobeniusNormRot(rot_opt, gt_pose_->block<3, 3>(0, 0));
        std::cout << "current g.t. pose inside LHM: " << std::endl;
        std::cout << *gt_pose_ << std::endl; 
        std::cout << "first estimated rotation difference: " << std::endl;
        std::cout << matrix_norm << std::endl;
        LHMEstimator::setFirstFrob(matrix_norm);
    }

    std::cout << "first estimated matrix by DRaM is: " << std::endl;
    std::cout << rot_opt << std::endl;
    if(bi_correct)
        return true;
    else    
        std::cerr << "Bar-Itzhack correction failed!" << std::endl;
        return false;
}

void LHMEstimator::GetCentroid(const std::vector<Eigen::Vector3d>& points3D0,
                               const std::vector<Eigen::Vector3d>& points3D1,
                               Eigen::Vector3d& pc, Eigen::Vector3d& qc) {
    // Compute centroids
    for (const auto& p : points3D0) {
        pc += p;
    }
    for (const auto& q : points3D1) {
        qc += q;
    }
    pc /= points3D0.size();
    qc /= points3D1.size();
}

void LHMEstimator::setGroundTruthPose(Eigen::Matrix3x4d* gt_pose) {
    if(gt_pose != nullptr)
        LHMEstimator::gt_pose_ = gt_pose;
}

void LHMEstimator::setGlobalOptions(const LHMOptions& options) {
    LHMEstimator::options_ = options;
}

void LHMEstimator::setFirstFrob(const double frob) {
    LHMEstimator::first_estimated_frob = frob;
}