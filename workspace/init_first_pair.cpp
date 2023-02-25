#include <string>
#include <Eigen/Core>
#include "optim/ransac.h"
#include "estimators/pose.h"
#include "base/image.h"
#include "base/camera.h"
#include "base/triangulation.h"
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

    std::cout << "size of match: " << matches.size() << std::endl;
    std::cout << "size of key1: " << key_points1.size() << std::endl;
    
    std::vector<Eigen::Vector2d> key_vec1 = SIFTPtsToVec(key_points1);
    colmap::Image cmp_image1 = SIFTtoCOLMAPImage(1, key_vec1, camera);
    std::vector<Eigen::Vector2d> key_vec2 = SIFTPtsToVec(key_points2);
    colmap::Image cmp_image2 = SIFTtoCOLMAPImage(2, key_vec2, camera);

    std::cout << "size of key_vec1: " << key_vec1.size() << std::endl;
    //register the first frame as identity mat and zeo trans
    Eigen::Vector4d qvec1 = Eigen::Vector4d(1, 0, 0, 0);
    Eigen::Vector3d tvec1 = Eigen::Vector3d::Zero();
    cmp_image1.SetQvec(qvec1);
    cmp_image1.SetTvec(tvec1);

    //collect matched vector before relative estimation
    //as the ransac estimator requires both vectors have same size
    std::unordered_map<int,int> vec2d1_idx_map; //map of idx of triangulation vector
    std::unordered_map<int,int> vec2d2_idx_map; //to the idx of original vector
    std::vector<Eigen::Vector2d> matched_vec1;
    std::vector<Eigen::Vector2d> matched_vec2;

    for (int i = 0; i < matches.size(); i++){
        int curr_idx1 = matches[i].first;
        int curr_idx2 = matches[i].second;
        vec2d1_idx_map[i] = curr_idx1;
        vec2d2_idx_map[i] = curr_idx2;
        matched_vec1.push_back(key_vec1[curr_idx1]);
        matched_vec2.push_back(key_vec2[curr_idx2]);
    }

    //start relative pose estimation and register pose for the frame 2
    colmap::RANSACOptions ransac_options = colmap::RANSACOptions();
    ransac_options.max_error = 0.05;
    Eigen::Vector4d qvec2 = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d tvec2 = Eigen::Vector3d::Zero();
    size_t num_inliers = 
        colmap::EstimateRelativePose(ransac_options, matched_vec1, matched_vec2, 
                                     &qvec2, &tvec2);
    cmp_image2.SetQvec(qvec2);
    cmp_image2.SetTvec(tvec2);

    //start triangulation
    Eigen::Matrix3d calibration = camera.CalibrationMatrix();
    Eigen::Matrix3x4d extrinsic_mat1 = cmp_image1.ProjectionMatrix(); //under proj.cc, only compose extrinsic
    Eigen::Matrix3x4d extrinsic_mat2 = cmp_image2.ProjectionMatrix();

    Eigen::Matrix3x4d proj_mat1 = calibration*extrinsic_mat1;
    Eigen::Matrix3x4d proj_mat2 = calibration*extrinsic_mat2;

    std::vector<Eigen::Vector3d> triangulate_3d = colmap::TriangulatePoints(proj_mat1, proj_mat2,
                                                                    matched_vec1, matched_vec2);

    int curr_3d_len = 0;
    for (int i = 0; i < triangulate_3d.size(); i++){
        int orig_idx1 = vec2d1_idx_map[i];
        int orig_idx2 = vec2d2_idx_map[i];
        //all 3d points are new
        int new_3d_id = curr_3d_len + 1;
        curr_3d_len++;
        global_3d_map[new_3d_id] = triangulate_3d[i];
        std::cout << i << " 's points of the first pair is: " << triangulate_3d[i] << std::endl;
        cmp_image1.SetPoint3DForPoint2D(orig_idx1,new_3d_id);
        cmp_image2.SetPoint3DForPoint2D(orig_idx2,new_3d_id);
        
    }
    global_image_map[1] = cmp_image1;
    global_image_map[2] = cmp_image2;
    global_keypts_map[1] = key_points1;
    global_keypts_map[2] = key_points2;
    std::cout << "num of 3d points from first pair is: " << 
           global_3d_map.size() << std::endl;
}