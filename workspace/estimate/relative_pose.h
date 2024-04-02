#include <vector>
#include <Eigen/Core>

#include "base/camera.h"
#include "base/camera_models.h"
#include "optim/ransac.h"
#include "optim/loransac.h"

/**
 * @brief COLMAP's relative pose, i.e. 5-points essential matrix
 * but with inlier mask, here the two 2D point sets should in
 * the camera space, i.e. after applying ImageToWorld
 * see TwoViewGeometry::EstimateCalibrated
*/
size_t RelativePoseWMask(const colmap::RANSACOptions& ransac_options,
                         colmap::Camera& camera,
                         const std::vector<Eigen::Vector2d>& points1,
                         const std::vector<Eigen::Vector2d>& points2,
                         Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                         std::vector<char>* inlier_mask);
