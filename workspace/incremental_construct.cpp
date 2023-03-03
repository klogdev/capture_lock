#include "sfm/incremental_mapper.h"
#include "sfm/incremental_triangulator.h"
#include "base/image.h"
#include "base/camera.h"
#include "base/triangulation.h"
#include "estimators/pose.h"

#include "feature/image_sift.h"
#include "feature/sift.h"

#include "incremental_construct.h"

colmap::Image SIFTtoCOLMAPImage(int image_id, std::vector<Eigen::Vector2d> features,
                                const colmap::Camera& camera){
    colmap::Image curr_image;
    curr_image.SetImageId(image_id); //need type conversion?
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
        Eigen::Vector2d curr_vec(key_points[i].i,key_points[i].j);
        key_vec.push_back(curr_vec);
    }
    return key_vec;
}

void IncrementOneImage(std::string image_path, int next_id,
                        colmap::Image& last_image,
                        colmap::Camera& camera,
                        std::unordered_map<int,colmap::Image>& global_image_map,
                        std::unordered_map<int,std::vector<sift::Keypoint>>& global_keypts_map,
                        std::unordered_map<int,Eigen::Vector3d>& global_3d_map
                        ){
    Image new_image(image_path);
    std::vector<sift::Keypoint> curr_key_points = GetKeyPoints(new_image);
    //covert sift keypts to eigen, should we only pick matched 2d pts for pose est??
    std::vector<Eigen::Vector2d> curr_keypts_vec = SIFTPtsToVec(curr_key_points);
    colmap::Image new_cmp_image = SIFTtoCOLMAPImage(next_id, curr_keypts_vec, camera);
    int last_id = next_id - 1;

    std::vector<sift::Keypoint> last_key_points = global_keypts_map[last_id];
    std::vector<Eigen::Vector2d> last_keypts_vec = SIFTPtsToVec(last_key_points);
    std::vector<std::pair<int, int>> matches = sift::find_keypoint_matches(last_key_points, curr_key_points);

    std::vector<Eigen::Vector3d> matched3d_from2d;
    std::vector<Eigen::Vector2d> matched2d_curr;
    for (int i = 0; i < matches.size(); i++){
        //assume key_points id of last image are consistent with the id
        //registered in points2d
        colmap::point2D_t last_2d_id = matches[i].first; //need type conversion
        colmap::Point2D last_2d = last_image.Point2D(last_2d_id);
        if (!last_2d.HasPoint3D()){
            continue;
        }

        colmap::point3D_t global_3d_key = last_2d.Point3DId();
        Eigen::Vector3d from2d_to3d = global_3d_map[global_3d_key];//need type conversion?
        matched3d_from2d.push_back(from2d_to3d);
        
        colmap::point2D_t curr_2d_id = matches[i].second;
        matched2d_curr.push_back(curr_keypts_vec[curr_2d_id]);
    }

    //collect matched vector before absolute estimation
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
        matched_vec1.push_back(last_keypts_vec[curr_idx1]);
        matched_vec2.push_back(curr_keypts_vec[curr_idx2]);
    }

    //start absolute pose estimation
    colmap::AbsolutePoseEstimationOptions absolute_options = colmap::AbsolutePoseEstimationOptions();
    absolute_options.ransac_options.max_error = 1.5;
    Eigen::Vector4d qvec_abs = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d tvec_abs = Eigen::Vector3d::Zero();
    std::vector<char> inlier_mask;
    size_t inliers = matched3d_from2d.size(); //need check def
    bool abs_pose = colmap::EstimateAbsolutePose(absolute_options, matched2d_curr,
                                                matched3d_from2d, &qvec_abs, &tvec_abs,
                                                &camera, &inliers, &inlier_mask);
    new_cmp_image.SetQvec(qvec_abs);
    new_cmp_image.SetTvec(tvec_abs);
    std::cout << "number of 2d-3d pairs in " << next_id << " is: " 
                << inliers << std::endl;
    std::cout << "result of " << next_id << " 's pos estimation" 
                << " is: " << abs_pose << std::endl;

    //start triangulation
    Eigen::Matrix3d calibration = camera.CalibrationMatrix();
    Eigen::Matrix3x4d extrinsic_mat1 = last_image.ProjectionMatrix(); //under proj.cc, only compose extrinsic
    Eigen::Matrix3x4d extrinsic_mat2 = new_cmp_image.ProjectionMatrix();

    Eigen::Matrix3x4d proj_mat1 = calibration*extrinsic_mat1;
    Eigen::Matrix3x4d proj_mat2 = calibration*extrinsic_mat2;

    //idx of triangulated pts are consistent with matched_vec,
    //they should be matched back to their original idx of matched vec
    std::vector<Eigen::Vector3d> triangulate_3d = colmap::TriangulatePoints(proj_mat1, proj_mat2,
                                                                    matched_vec1, matched_vec2);
    
    std::cout << "num of 3d in " << next_id << " image is: " << triangulate_3d.size() << std::endl; 
    std::cout << "num of 2d in " << next_id << " image is: " << matched_vec1.size() << std::endl; 
    int curr_3d_len = global_3d_map.size();
    for (int i = 0; i < triangulate_3d.size(); i++){
        int orig_idx1 = vec2d1_idx_map[i];
        int orig_idx2 = vec2d2_idx_map[i];
        if (last_image.Point2D(orig_idx1).HasPoint3D()){
            colmap::point3D_t curr3d_id = last_image.Point2D(orig_idx1).Point3DId();//type conversion??
            //overlap the 3d point coord by the updated one
            global_3d_map[curr3d_id] = triangulate_3d[i];
            new_cmp_image.SetPoint3DForPoint2D(orig_idx2,curr3d_id);
        }
        else {
            int new_3d_id = curr_3d_len + 1;
            curr_3d_len++;
            global_3d_map[new_3d_id] = triangulate_3d[i];
            last_image.SetPoint3DForPoint2D(orig_idx1,new_3d_id);
            new_cmp_image.SetPoint3DForPoint2D(orig_idx2,new_3d_id);
        }
    }
    global_image_map[next_id] = new_cmp_image;
    global_keypts_map[next_id] = curr_key_points;
}
