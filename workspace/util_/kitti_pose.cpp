#include <Eigen/Core>

#include "base/pose.h"

void QuatTransFromExtrinsic(Eigen::Vector4d& quat, Eigen::Vector3d& trans,
                            const Eigen::Matrix3x4d& extrinsic){
    Eigen::Matrix3d rot = extrinsic.block<3, 3>(0, 0);
    trans = extrinsic.col(3);
    quat = colmap::RotationMatrixToQuaternion(rot);
}

void InverseExtrinsic(const Eigen::Matrix3x4d& kitti_pose, 
                      Eigen::Matrix3x4d& extrinsic) {
    // Extract rotation R and translation t
    Eigen::Matrix3d R = kitti_pose.block<3, 3>(0, 0);
    Eigen::Vector3d t = kitti_pose.col(3);

    // manual conversion, from the rotation
    // that estimated by svd of t_kitti to t_est
    Eigen::Matrix3d convert;

    convert << -0.91828087, -0.02930214, 0.39484381,
    0.08722898, -0.9877266, 0.12956568,
    0.38620118, 0.15341951, 0.90956644;

    // Compute inverse rotation R-transpose and inverse translation (-R^T * t)
    Eigen::Matrix3d R_inv = R.transpose();
    Eigen::Vector3d t_inv = -R_inv * convert * t;

    // Construct the extrinsic matrix
    extrinsic.block<3, 3>(0, 0) = R_inv * convert;
    extrinsic.col(3) = t_inv;
}