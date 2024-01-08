#include <Eigen/Core>

#include "base/pose.h"

void QuatTransFromExtrinsic(Eigen::Vector4d& quat, Eigen::Vector3d& trans,
                            Eigen::Matrix3x4d& extrinsic){
    Eigen::Matrix3d rot = extrinsic.block<3, 3>(0, 0);
    trans = extrinsic.col(3);
    quat = colmap::RotationMatrixToQuaternion(rot);
}