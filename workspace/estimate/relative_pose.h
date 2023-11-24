#include <vector>
#include <Eigen/Core>

#include "base/camera.h"
#include "base/camera_models.h"
#include "optim/loransac.h"

// adopts COLMAP's relative pose but with inlier masks
size_t RelativePoseWMask(const RANSACOptions& ransac_options,
                        const std::vector<Eigen::Vector2d>& points1,
                        const std::vector<Eigen::Vector2d>& points2,
                        Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                        std::vector<char>* inlier_mask);