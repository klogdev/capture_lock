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

std::unordered_map<std::pair<int,int>, sift::Keypoint> SIFTPtsToHash(std::vector<sift::Keypoint> key_points){
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

std::vector<Eigen::Vector3d> GetPoint3Dfrom2D(int image_id, 
                                            std::vector& global_image_map){
    colmap::Image corr_image = global_image_map[image_id];

}

std::vector<colmap::Point3D> IncrementOneImage(std::string image_path,
                                            int next_id,
                                            const& colmap::Image last_image,
                                            const& colmap::Camera camera,
                                            std::unordered_map<int,colmap::Image>& global_image_map,
                                            std::unordered_map<int,Eigen::Vector3d>& global_keypts_map){
    Image new_image(image_path);
    std::vector<sift::Keypoint> curr_key_points = sift::find_keypoints_and_descriptors(new_image);
    colmap::Image new_cmp_image = SIFTtoCOLMAPImage(next_id, curr_keypts, camera);
    int last_id = next_id - 1;

    std::vector<sift::Keypoint> last_key_points = global_keypts_id[last_id];
    std::vector<std::pair<int, int>> matches = sift::find_keypoint_matches(last_key_points, curr_key_points);
    //covert sift keypts to eigen, should we only pick matched 2d pts for pose est??
    std::vector<Eigen::Vector2d> curr_keypts_vec = SIFTPtstoVec(curr_key_points);

    std::vector<Eigen::Vector3d> matched3d_from2d;
    for (int i = 0; i < matches.size(); i++){
        //assume key_points id for last image are consistent with the id
        //registered in points2d
        colmap::point2D_t curr_2d_id = matches[i].first; //need type conversion
        colmap::Point2D curr_2d = last_image.Point2D(curr_2d_id);
        colmap::point3D_t global_3d_key = curr_2d.Point3DId();
        Eigen::Vector3d from2d_to3d = global_keypts_map[global_3d_key];
        matched3d_from2d.push_back(from2d_to3d);
    }

    colmap::AbsolutePoseEstimationOptions absolute_options = colmap::AbsolutePoseEstimationOptions();
    absolute_options.ransac_options.max_error = 0.05;
    Eigen::Vector4d qvec_abs = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d tvec_abs = Eigen::Vector3d::Zero();
    std::vector<char> inlier_mask;
    bool abs_pose = colmap::EstimateAbsolutePose(absolute_options, curr_keypts_vec,
                                                matched3d_from2d, &qvec_abs, &tvec_abs,
                                                &camera1, &inliers, &inlier_mask);
    new_image.SetQvec(qvec_abs);
    new_image.SetTvec(tvec_abs);

}
