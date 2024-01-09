#include <Eigen/Core>

#include "base/pose.h"

/**
 * @brief helper fn to decompose the read extrinsic matrix to 
 * a quaternion and a translation vector
 * @arg quat, trans: the pose to be decomposed
*/
void QuatTransFromExtrinsic(Eigen::Vector4d& quat, Eigen::Vector3d& trans,
                            const Eigen::Matrix3x4d& extrinsic);