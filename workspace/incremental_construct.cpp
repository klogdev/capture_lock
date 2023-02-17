#include "sfm/incremental_mapper.h"
#include "sfm/incremental_triangulator.h"
#include "base/image.h"
#include "base/camera.h"

#include "feature/image_sift.h"
#include "feature/sift.h"

Image LoadImage(const std::string folder, const std::string image_name){
    std::string abs_path = folder + image_name;
    Image image(abs_path);
    return image;
}

colmap::Image SIFTtoCOLMAPImage(int image_id, std::vector<Eigen::Vector2d> features,
                                colmap::Camera camera){
    colmap Image curr_image;
    curr_image.SetImageId(image_id);
    curr_image.SetCamera(camera);
    curr_image.SetPoints2D(features);
    return curr_image;
}