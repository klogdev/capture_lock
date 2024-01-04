#include "base/image.h"

#include "feature/image_sift.h"
#include "feature/sift.h"

colmap::Image SIFTtoCOLMAPImage(int image_id, std::vector<Eigen::Vector2d> features,
                                const colmap::Camera& camera){
    colmap::Image curr_image;
    curr_image.SetImageId(image_id); //automatic type conversion to the built-in type
    curr_image.SetCameraId(camera.CameraId());
    curr_image.SetPoints2D(features);
    return curr_image;
}

std::vector<sift::Keypoint> GetKeyPoints(Image& image){
    const Image& image1 = image.channels == 1 ? image1 : rgb_to_grayscale(image);
    std::vector<sift::Keypoint> key_points = sift::find_keypoints_and_descriptors(image1);
    return key_points;
}

std::vector<Eigen::Vector2d> SIFTPtsToVec(std::vector<sift::Keypoint> key_points){
    std::vector<Eigen::Vector2d> key_vec;
    for (int i = 0; i < key_points.size(); i++){
        Eigen::Vector2d curr_vec(key_points[i].i, key_points[i].j);
        key_vec.push_back(curr_vec);
    }
    return key_vec;
}
