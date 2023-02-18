#include "sfm/incremental_mapper.h"
#include "sfm/incremental_triangulator.h"
#include "base/image.h"
#include "base/camera.h"
#include "base/triangulation.h"

#include "feature/image_sift.h"
#include "feature/sift.h"

#include "incremental_construct.h"

Image LoadImage(const std::string folder, const std::string image_name){
    std::string abs_path = folder + image_name;
    Image image(abs_path);
    return image;
}

colmap::Image SIFTtoCOLMAPImage(int image_id, std::vector<Eigen::Vector2d> features,
                                colmap::Camera camera){
    colmap::Image curr_image;
    curr_image.SetImageId(image_id);
    curr_image.SetCamera(camera);
    curr_image.SetPoints2D(features);
    return curr_image;
}

std::vector<sift::Keypoint> GetKeyPoints(Image& image){
    const Image& image1 = image.channels == 1 ? image1 : rgb_to_grayscale(image);
    std::vector<sift::Keypoint> key_points = sift::find_keypoints_and_descriptors(image1);
    return key_points;
}

std::vector<Eigen::Vector2d> SIFTPtsToVector(std::vector<sift::Keypoint> key_points){
    std::vector<Eigen::Vector2d> key_vec;

    for(int i = 0; i < key_points.size(); i++){
        int key_idx = key_points[i];
        
        sift::Keypoint CurrKey1 = KeyPoints1[KeyIdx1];
        sift::Keypoint CurrKey2 = KeyPoints2[KeyIdx2];
        Eigen::Vector2d Pt1(CurrKey1.x, CurrKey1.y);
        Eigen::Vector2d Pt2(CurrKey2.x, CurrKey2.y);
}

std::vector<colmap::Point3D> IncrementOneImage(std::string image_path, 
                                            const& colmap::Image){
    Image new_image(image_path);
}