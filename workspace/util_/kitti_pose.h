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

/**
 * @brief calculate relative trans from g.t.'s world to cam1, 2 poses
 * t^{c1->c2} = t^{w->c2} - R^{w->c1}*t^{w->c1}
*/
void RelativeTransFromGT(const Eigen::Vector4d& quat1, const Eigen::Vector3d& trans1,
                         const Eigen::Vector3d& trans2, Eigen::Vector3d& rel_t);