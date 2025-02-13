#include <map>

#include "sfm/incremental_mapper.h"
#include "sfm/incremental_triangulator.h"

#include "base/image.h"
#include "base/point3d.h"
#include "base/camera.h"
#include "base/triangulation.h"
#include "base/track.h"
#include "base/projection.h"

#include "estimators/pose.h"

#include "util/math.h"

#include "feature/image_sift.h"
#include "feature/sift.h"

#include "util_/reprojection.h"
#include "util_/triangulate.h"
#include "util_/sift_colmap.h"
#include "util_/pnp_lone.h"

#include "incremental_construct.h"
#include "estimate/relative_pose.h"

void IncrementOneImage(std::string image_path, int new_id,
                       int last_id, colmap::Camera& camera,
                       std::unordered_map<int, colmap::Image>& global_image_map,
                       std::unordered_map<int, std::vector<sift::Keypoint>>& global_keypts_map,
                       std::unordered_map<int, colmap::Point3D>& global_3d_map,
                       std::map<int, std::pair<Eigen::Vector4d, Eigen::Vector3d>> gt_map,
                       int resize_w, int resize_h) {
                            
    Image new_image(image_path, resize_w, resize_h);
    std::vector<sift::Keypoint> curr_key_points = GetKeyPoints(new_image);
    // convert sift keypts to eigen, 
    // will pick inlier matches by ransac via essential mat model
    // can use epipole model if we have gyro readings
    std::vector<Eigen::Vector2d> curr_keypts_vec = SIFTPtsToVec(curr_key_points);
    colmap::Image new_cmp_image = SIFTtoCOLMAPImage(new_id, curr_keypts_vec, camera);
    colmap::Image last_image = global_image_map[last_id];

    std::vector<sift::Keypoint> last_key_points = global_keypts_map[last_id];
    std::vector<Eigen::Vector2d> last_keypts_vec = SIFTPtsToVec(last_key_points);
    std::vector<std::pair<int, int>> matches = sift::find_keypoint_matches(last_key_points, curr_key_points);

    std::cout << "num of features in " << new_id << " is: " << curr_key_points.size() << std::endl;
    std::cout << "num of features in " << last_id << " is: " << last_key_points.size() << std::endl;
    std::cout << "num of raw matches between " << last_id << " and " << new_id << " is: " << matches.size() << std::endl;

    // collect matched vector with relative pose w/ RANSAC
    // as a pre-filtering
    std::unordered_map<int,int> vec2d1_idx_map; // map of idx of matched vec for relative pose
    std::unordered_map<int,int> vec2d2_idx_map; // to the idx of original feature vector
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

    // start relative pose estimation w/ RANSAC for a pre-filtering
    colmap::RANSACOptions ransac_options = colmap::RANSACOptions();
    ransac_options.max_error = 1.0;
    Eigen::Vector4d qvec_rel = Eigen::Vector4d(0, 0, 0, 1); // init relative pose
    Eigen::Vector3d tvec_rel = Eigen::Vector3d::Zero();     // w/ 0 rot and trans
    std::vector<char> inlier_mask_rel;
    std::vector<Eigen::Vector3d> point_rel;
    // use customized relative pose estimator w/ inlier masks
    size_t num_inliers = 
        RelativePoseWMask(ransac_options, camera, matched_vec1, matched_vec2, 
                                    &qvec_rel, &tvec_rel, &inlier_mask_rel, &point_rel);

    int test_inliers = 0; // for Debugging
    int inlier_repro = 0;

    // we don't need to record the original vector id 
    // as the PnP only estimate the inliers as an intermediate step
    std::vector<Eigen::Vector3d> matched3d_from2d;
    std::vector<Eigen::Vector2d> matched2d_curr;

    for (int i = 0; i < matches.size(); i++){

        if(inlier_mask_rel[i] == 0)
            continue;
        
        test_inliers++;

        // assume sift::key_points' id of last image are consistent with the id
        // that registered in points2d
        colmap::point2D_t last_2d_id = matches[i].first; // automatic type conversion
        colmap::Point2D last_2d = last_image.Point2D(last_2d_id); // here 2D is the pixel coordinate
        if (!last_2d.HasPoint3D()){
            continue;
        }

        colmap::point3D_t global_3d_key = last_2d.Point3DId();
        Eigen::Vector3d from2d_to3d = global_3d_map[global_3d_key].XYZ(); // extract vector from point3d's attr
        matched3d_from2d.push_back(from2d_to3d);

        Eigen::Vector2d last_2d_pt = last_2d.XY();
        
        double repro_err = colmap::CalculateSquaredReprojectionError(last_2d_pt,
                                                                     from2d_to3d,
                                                                     last_image.Qvec(),
                                                                     last_image.Tvec(),
                                                                     camera);
        if(repro_err <= 0.8)
            inlier_repro++;
        std::cout << "the reprojection error of image " << last_id 
        << "'s point " << last_2d_id << " is " << repro_err << std::endl; 
        
        colmap::point2D_t curr_2d_id = matches[i].second;
        // a normalized point for the PnP estimation will be converted 
        // inside the lone PnP function
        matched2d_curr.push_back(curr_keypts_vec[curr_2d_id]);
    }

    // num of inliers after ransac filtering
    std::cout << "num of inlier matches between " << last_id << " and " << new_id << " is: " << test_inliers << std::endl;
    std::cout << "num of matched 3D in image " << new_id << " is: " << matched3d_from2d.size() << std::endl;
    std::cout << "rate of reprojection inlier " << last_id << " is: " << 
    (float)inlier_repro/matched3d_from2d.size() << std::endl;

    // start absolute pose estimation
    // colmap::AbsolutePoseEstimationOptions absolute_options = colmap::AbsolutePoseEstimationOptions();
    // absolute_options.ransac_options.max_error = 1.0;
    // Eigen::Vector4d qvec_abs = Eigen::Vector4d(0, 0, 0, 1);
    // Eigen::Vector3d tvec_abs = Eigen::Vector3d::Zero();
    // std::vector<char> inlier_mask;
    // size_t inliers = matched3d_from2d.size(); 
    // bool abs_pose = colmap::EstimateAbsolutePose(absolute_options, matched2d_curr,
    //                                             matched3d_from2d, &qvec_abs, &tvec_abs,
    //                                             &camera, &inliers, &inlier_mask);


    // start absolute pose estimation via lone EPnP
    colmap::RANSACOptions options_epnp = colmap::RANSACOptions();
    options_epnp.max_error = 0.8;
    Eigen::Vector4d qvec_epnp = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d tvec_epnp = Eigen::Vector3d::Zero();
    std::vector<char> inlier_mask_epnp;
    size_t inliers = matched3d_from2d.size(); 

    bool epnp_pose = PnPEstimation(options_epnp, matched2d_curr,
                                   matched3d_from2d, &qvec_epnp, &tvec_epnp,
                                   &camera, &inliers, &inlier_mask_epnp);
    std::cout << "number of inliers 2d-3d pairs by customized EPnP in " << new_id << " is: " 
                << inliers << std::endl;
    std::cout << "result of " << new_id << " 's EPnP estimation" 
                << " is: " << std::endl;
    std::cout << qvec_epnp << std::endl;
    std::cout << tvec_epnp << std::endl;

    // use g.t. poses if we specified in the map, now we register pose
    // that estimated by epnp
    if(gt_map.find(new_id) != gt_map.end()){
        new_cmp_image.SetQvec(gt_map[new_id].first);
        new_cmp_image.SetTvec(gt_map[new_id].second);
    }
    else{
        new_cmp_image.SetQvec(qvec_epnp);
        new_cmp_image.SetTvec(tvec_epnp);
    }
    // start triangulation

    // idx of triangulated pts are consistent with matched_vec,
    // they should be matched back to their original idx of feature vec
    // here we use all matched point (without RANSAC) for triangulation
    // but filter out outliers when we register to the 3d global map
    std::vector<Eigen::Vector3d> triangulate_3d;
    TriangulateImage(last_image, new_cmp_image, camera, matched_vec1, matched_vec2,
                     triangulate_3d);
    
    double min_ang = colmap::DegToRad(1.5); // follow colmap's default value                                      
    int curr_3d_len = global_3d_map.size();
    for (int i = 0; i < triangulate_3d.size(); i++){
        if(inlier_mask_rel[i] == 0)
            continue;
        // check the quality of the triangulation
        if(!CheckTriangulateQuality(last_image.ProjectionMatrix(),
                                    new_cmp_image.ProjectionMatrix(),
                                    last_image.ProjectionCenter(),
                                    new_cmp_image.ProjectionCenter(),
                                    triangulate_3d[i], min_ang)){
            continue;
        }
        int orig_idx1 = vec2d1_idx_map[i];
        int orig_idx2 = vec2d2_idx_map[i];
        if (last_image.Point2D(orig_idx1).HasPoint3D()) {
            colmap::point3D_t curr3d_id = last_image.Point2D(orig_idx1).Point3DId();// type conversion??
            // overlap the 3d point coord by the updated one
            colmap::Point3D& curr_3d = global_3d_map[curr3d_id];
            curr_3d.SetXYZ(triangulate_3d[i]);
            curr_3d.Track().AddElement(new_id, orig_idx2);
            new_cmp_image.SetPoint3DForPoint2D(orig_idx2, curr3d_id);
        }
        else {
            int new_3d_id = curr_3d_len; // the id is 0-indexed
            curr_3d_len++;
            colmap::Point3D new_3d;
            new_3d.SetXYZ(triangulate_3d[i]);
            new_3d.Track().AddElement(last_id, orig_idx1);
            new_3d.Track().AddElement(new_id, orig_idx2);
            global_3d_map[new_3d_id] = new_3d;

            last_image.SetPoint3DForPoint2D(orig_idx1, new_3d_id);
            new_cmp_image.SetPoint3DForPoint2D(orig_idx2, new_3d_id);
        }
    }
    global_image_map[new_id] = new_cmp_image;
    global_keypts_map[new_id] = curr_key_points; // sift Keypoint
}
