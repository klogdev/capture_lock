#ifndef ESTIMATE_RELATIVE_POSE_H_
#define ESTIMATE_RELATIVE_POSE_H_

#include <vector>
#include <Eigen/Core>

#include "base/camera.h"
#include "base/camera_models.h"
#include "optim/ransac.h"
#include "optim/loransac.h"

/**
 * @brief the implementation of COLMAP's relative pose, i.e. 5-points essential matrix
 * but with inlier mask, 
 * here the two 2D point sets are pixel points
 * the 5-points need the camera space points, i.e. after applying ImageToWorld
 * see TwoViewGeometry::EstimateCalibrated,
 * we convert pixel to homogeneaous points inside the function
*/
size_t RelativePoseWMask(const colmap::RANSACOptions& ransac_options,
                         colmap::Camera& camera,
                         const std::vector<Eigen::Vector2d>& points1,
                         const std::vector<Eigen::Vector2d>& points2,
                         Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                         std::vector<char>* inlier_mask,
                         std::vector<Eigen::Vector3d>* points3D);

#endif // ESTIMATE_RELATIVE_POSE_H_