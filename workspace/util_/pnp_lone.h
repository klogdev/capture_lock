#include <Eigen/Core>

#include "base/image.h"
#include "base/camera.h"
#include "estimators/absolute_pose.h"
#include "optim/ransac.h"

/**
 * @brief customized PnP estimator to directly apply each estimator
 * skip using the EstimateAbsolutePose from COLMAP
*/
bool PnPEstimation(const colmap::RANSACOptions& options,
                   const std::vector<Eigen::Vector2d>& points2D,
                   const std::vector<Eigen::Vector3d>& points3D,
                   Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                   colmap::Camera* camera, size_t* num_inliers,
                   std::vector<char>* inlier_mask);