#include <Eigen/Core>

#include "base/pose.h"

/**
 * @brief helper fn to decompose the read extrinsic matrix to 
 * a quaternion and a translation vector
 * @arg quat, trans: the pose to be decomposed
*/
void QuatTransFromExtrinsic(Eigen::Vector4d& quat, Eigen::Vector3d& trans,
                            const Eigen::Matrix3x4d& extrinsic);

/**
 * @brief inverse the extrinsic matrix to a kitti pose or vice versa by
 * converting [R|t] to [-R^T|-R^T*t]
*/
void InverseExtrinsic(const Eigen::Matrix3x4d& kitti_pose, 
                      Eigen::Matrix3x4d& extrinsic);