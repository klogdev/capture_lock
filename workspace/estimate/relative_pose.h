#include <vector>
#include <Eigen/Core>

#include "base/camera.h"
#include "base/camera_models.h"
#include "optim/ransac.h"
#include "optim/loransac.h"

/**
 * @brief COLMAP's relative pose, i.e. 5-points essential matrix
 *  but with inlier mask
*/
size_t RelativePoseWMask(const colmap::RANSACOptions& ransac_options,
                        const std::vector<Eigen::Vector2d>& points1,
                        const std::vector<Eigen::Vector2d>& points2,
                        Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                        std::vector<char>* inlier_mask);

void TestEigen(std::vector<Eigen::Matrix3x4d>& test);