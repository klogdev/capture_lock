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
                                const& colmap::Camera camera){
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

std::unordered_map<std::pair<int,int>, sift::keypoint> SIFTPtsToHash(std::vector<sift::Keypoint> key_points){
    std::unordered_map<std::pair<int,int>, sift::keypoint> key_map;

    for(int i = 0; i < key_points.size(); i++){
        std::pair<int,int> curr_idx;
        curr_idx.first = key_points[i].i;
        curr_idx.second = key_points[i].j;
        key_map[curr_idx] = key_points[i];
    }
    return key_map;
}

std::vector<Eigen::Vector2d> SIFTPtsToVec(std::vector<sift::Keypoint> key_points){
    std::vector<Eigen::Vector2d> key_vec;
    for (int i = 0; i < key_points.size(); i++){
        Eigen::Vector2d curr_vec(key_points[i].i,key_points[i].j);
    }
    return key_vec;
}

std::vector<colmap::Point3D> IncrementOneImage(std::string image_path,
                                            int next_id,
                                            const& colmap::Image last_image,
                                            const& colmap::Camera camera){
    Image new_image(image_path);
    std::vector<sift::Keypoint> curr_keypts = sift::find_keypoints_and_descriptors(new_image);
    colmap::Image new_cmp_image = SIFTtoCOLMAPImage(next_id, curr_keypts, camera);

}
