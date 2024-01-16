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
    // Extract rotation (R) and translation (t)
    Eigen::Matrix3d R = kitti_pose.block<3, 3>(0, 0);
    Eigen::Vector3d t = kitti_pose.col(3);

    // Compute inverse rotation (R transpose) and inverse translation (-R^T * t)
    Eigen::Matrix3d R_inv = R.transpose();
    Eigen::Vector3d t_inv = -R_inv * t;

    // Construct the extrinsic matrix
    extrinsic.block<3, 3>(0, 0) = R_inv;
    extrinsic.col(3) = t_inv;
}