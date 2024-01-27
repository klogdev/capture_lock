#include <Eigen/Core>
#include <limits>

#include "estimate/lhm.h"

bool LHMEstimator::ComputeLHMPose(const std::vector<Eigen::Vector2d>& points2D,
                                  const std::vector<Eigen::Vector3d>& points3D,
                                  Eigen::Matrix3x4d* proj_matrix) {
    int n_points = points2D.size();
    std::vector<Eigen::Matrix3d> V; // initialize a vector of line of sight matrices
    Eigen::Matrix3d sum_Vk; // summation of V in eqn. 20
    sum_Vk.setZero();

    std::vector<Eigen::Vector3d>& homogeneous_pts; // for later use in the weak perspective model

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
    double n_inv = 1.0 / npts;  // Inverse of the number of points

    // Compute Tfact directly from the eqn. 20
    Eigen::Matrix3d Tfact = (I - n_inv * sum_Vk).inverse() * n_inv;

    // Calculate the initial guess of rotation and translation
    Eigen::Matrix3d init_rot;
    Eigen::Vector3d init_trans;

    bool weak_persp = WeakPerspectiveQuat(points3D, homogeneous_pts,
                                          init_rot, init_trans); 
    // obatain the transaltion from initialized R
    bool init_pose = TransFromRotLHM(point3D, V, Tfact, init_rot, init_trans);

    int iter = 0;
    double curr_err = std::numeric_limits<double>::max();
    double old_err = -1.0; // dummy old error to start the while loop


    std::vector<Eigen::Vector3d> temp_rotated; // can be overwrite during iteration
    temp_rotated.resize(n_points);

    while(abs((old_err - curr_err)/old_err) > options_.lhm_tolerance && 
          && curr_err > options_.lhm_epsilon && iter < options_.lhm_iter) {
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
    Eigen::Vector3d pc = Eigen::Vector3d::Zero();
    Eigen::Vector3d qc = Eigen::Vector3d::Zero();

    // Compute centroids
    for (const auto& p : points3D0) {
        pc += p;
    }
    for (const auto& q : points3D1) {
        qc += q;
    }
    pc /= pts0.size();
    qc /= pts1.size();

    // Compute M from centered points
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < points3D0.size(); i++) {
        Eigen::Vector3d p = points3D0[i] - pc;
        Eigen::Vector3d q = points3D1[i] - qc;

        M += p * q.transpose(); // Outer product
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