#include <Eigen/Core>
#include <iostream>

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
    // add dummy conversion for a guessing of possible misaligment
    Eigen::Matrix3d R_inv = R.transpose();
    Eigen::Vector3d t_inv = -R_inv * t; //* convert * t;
    std::cout << "converted trans: " << std::endl;
    std::cout << t_inv << std::endl;

    // negate part of the components
    // t_inv(1) = -t_inv(1);
    // t_inv(2) = -t_inv(2);
    // std::cout << "negated trans: " << std::endl;
    // std::cout << t_inv << std::endl;

    // Construct the extrinsic matrix
    extrinsic.block<3, 3>(0, 0) = R_inv; 
    extrinsic.col(3) = t_inv;
}

void RelativeTransFromGT(const Eigen::Vector4d& quat1, const Eigen::Vector3d& trans1,
                         const Eigen::Vector3d& trans2, Eigen::Vector3d& rel_t) {
    Eigen::Matrix3d rot1 = colmap::QuaternionToRotationMatrix(quat1);
    rel_t = trans2 - rot1*trans1;
}