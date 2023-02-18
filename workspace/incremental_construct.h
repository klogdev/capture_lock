#include <Eigen/Core>
#include "feature/sift.h"
#include "feature/image_sift.h"
#include "base/image.h"
#include "base/point3d.h"
#include <string>
#include "feature/sift.h"

Image LoadImage(const std::string folder, const std::string image_name);

colmap::Image SIFTtoCOLMAPImage(int image_id, std::vector<Eigen::Vector2d> features,
                                const& colmap::Camera camera);

std::vector<colmap::Point3D> IncrementOneImage(std::string image_path, 
                                            const& colmap::Image);

std::vector<sift::Keypoint> GetKeyPoints(Image& image);

std::vector<Eigen::Vector2d> SIFTPtsToVector(std::vector<sift::Keypoint> key_points);

    