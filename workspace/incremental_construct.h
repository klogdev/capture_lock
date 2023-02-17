#include "feature/sift.h"
#include "feature/image_sift.h"
#include "base/image.h"
#include <string>

Image LoadImage(const std::string folder, const std::string image_name);

colmap::Image SIFTtoCOLMAPImage(int image_id, std::vector<Eigen::Vector2d> features,
                                colmap::Camera camera);