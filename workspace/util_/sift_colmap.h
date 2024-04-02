#include <Eigen/Core>

#include "feature/sift.h"
#include "feature/image_sift.h"
#include "base/image.h"

#include <string>

/**
 * @brief registered sift detected features into a new colmap::Image instance
*/
colmap::Image SIFTtoCOLMAPImage(int image_id, std::vector<Eigen::Vector2d> features,
                                const colmap::Camera& camera);

/**
 * @brief generate sift keypoints from feature/sift
*/
std::vector<sift::Keypoint> GetKeyPoints(Image& image);

/**
 * @brief convert i, j coord from the SIFT detected points 
 * to an eigen vector for the init of colmap::Image
*/
std::vector<Eigen::Vector2d> SIFTPtsToVec(std::vector<sift::Keypoint> key_points);