#include <string>
#include <Eigen/Core>
#include "optim/ransac.h"
#include "estimators/pose.h"
#include "base/image.h"
#include "base/camera.h"
#include "feature/sift.h"
#include "feature/image_sift.h"
#include "incremental_construct.h"

void InitFirstPair(const std::string first_path, const std::string second_path,
                    colmap::Camera& camera,
                    std::unordered_map<int,colmap::Image>& global_image_map,
                    std::unordered_map<int,std::vector<sift::Keypoint>>& global_keypts_map,
                    std::unordered_map<int,Eigen::Vector3d>& global_3d_map){
    //initialize the Image class by its path (feature/image_sift)
    Image image1(first_path);
    Image image2(second_path);

    std::vector<sift::Keypoint> key_points1 = GetKeyPoints(image1);
    std::vector<sift::Keypoint> key_points2 = GetKeyPoints(image2);
    std::vector<std::pair<int, int>> matches = sift::find_keypoint_matches(key_points1, key_points2);

    std::vector<Eigen::Vector2d> key_vec1 = SIFTPtsToVec(key_points1);
    colmap::Image cmp_image1 = SIFTtoCOLMAPImage(1, key_vec1, camera);
    std::vector<Eigen::Vector2d> key_vec2 = SIFTPtsToVec(key_points2);
    colmap::Image cmp_image2 = SIFTtoCOLMAPImage(2, key_vec2, camera);

    //register the first frame as identity mat and zeo trans
    Eigen::Vector4d qvec1 = Eigen::Vector4d(1, 0, 0, 0);
    Eigen::Vector3d tvec1 = Eigen::Vector3d::Zero();
    cmp_image1.SetQvec(qvec1);
    cmp_image1.SetTvec(tvec1);

    //start relative pose estimation and register pose for the frame 2
    colmap::RANSACOptions ransac_options = colmap::RANSACOptions();
    ransac_options.max_error = 0.05;
    Eigen::Vector4d qvec2 = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d tvec2 = Eigen::Vector3d::Zero();
    size_t num_inliers = 
        colmap::EstimateRelativePose(ransac_options, key_vec1, key_vec2, 
                                     qvec2, tvec2);
    cmp_image2.SetQvec(qvec2);
    cmp_image2.SetTvec(tvec2);
}