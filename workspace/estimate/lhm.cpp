#include <Eigen/Core>
#include "estimate/lhm.h"

bool LHMEstimator::ComputeLHMPose(const std::vector<Eigen::Vector2d>& points2D,
                                  const std::vector<Eigen::Vector3d>& points3D,
                                  Eigen::Matrix3x4d* proj_matrix) {
    int n_points = points2D.size();
    std::vector<Eigen::Matrix3d> V; // initialize a vector of line of sight matrices
    Eigen::Matrix3d sum_Vk; // summation of V in eqn. 20
    sum_Vk.setZero();

    std::vector<Eigen::Vector3d>& homogeneous_pts; // for later use in weak perspective model

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
}