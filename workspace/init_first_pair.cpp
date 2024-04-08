#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "optim/ransac.h"
#include "estimators/pose.h"
#include "base/image.h"
#include "base/camera.h"
#include "base/triangulation.h"
#include "base/projection.h"
#include "base/pose.h"

#include "feature/sift.h"
#include "feature/image_sift.h"

#include "estimate/relative_pose.h"

#include "util/math.h"

#include "util_/reprojection.h"
#include "util_/triangulate.h"
#include "util_/sift_colmap.h"
#include "util_/kitti_pose.h"
#include "util_/pnp_lone.h"

void InitFirstPair(const std::string first_path, const std::string second_path,
                   colmap::Camera& camera,
                   std::unordered_map<int,colmap::Image>& global_image_map,
                   std::unordered_map<int,std::vector<sift::Keypoint>>& global_keypts_map,
                   std::unordered_map<int,colmap::Point3D>& global_3d_map,
                   int resize_w, int resize_h, 
                   Eigen::Vector4d& gt_quat1, Eigen::Vector3d& gt_trans1,
                   Eigen::Vector4d& gt_quat2, Eigen::Vector3d& gt_trans2) {
    // initialize the Image class by its path (via lib: feature/image_sift)

    Image image1(first_path, resize_w, resize_h);
    Image image2(second_path, resize_w, resize_h);

    std::vector<sift::Keypoint> key_points1 = GetKeyPoints(image1);
    std::vector<sift::Keypoint> key_points2 = GetKeyPoints(image2);
    std::vector<std::pair<int, int>> matches = sift::find_keypoint_matches(key_points1, key_points2);

    std::cout << "debug num of matches in the first pair: " << matches.size() << std::endl;
    
    std::vector<Eigen::Vector2d> key_vec1 = SIFTPtsToVec(key_points1);
    colmap::Image cmp_image1 = SIFTtoCOLMAPImage(1, key_vec1, camera);
    std::vector<Eigen::Vector2d> key_vec2 = SIFTPtsToVec(key_points2);
    colmap::Image cmp_image2 = SIFTtoCOLMAPImage(2, key_vec2, camera);

    // register the first frame as the g.t.
    cmp_image1.SetQvec(gt_quat1);
    cmp_image1.SetTvec(gt_trans1);

    // collect matched vector before relative estimation
    // as the ransac estimator requires both vectors have same size
    std::unordered_map<int,int> vec2d1_idx_map; // the map of idx of matched vec used in relative pose
    std::unordered_map<int,int> vec2d2_idx_map; // to the idx of original features vector
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

    // start relative pose estimation and register pose for the frame 2
    colmap::RANSACOptions ransac_options = colmap::RANSACOptions();
    ransac_options.max_error = 0.8;
    Eigen::Vector4d qvec_init = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d tvec_init = Eigen::Vector3d::Zero();
    std::vector<char> inlier_mask_rel;
    // use customized relative pose estimator w/ inlier masks
    // we convert pixel points to normalized space inside the function
    size_t num_inliers = 
        RelativePoseWMask(ransac_options, camera, matched_vec1, matched_vec2, 
                                     &qvec_init, &tvec_init, &inlier_mask_rel);
    std::cout << "inliers from relative pose: " << num_inliers << std::endl;
    std::cout << "length of inlier masks: " << inlier_mask_rel.size() << std::endl;

    // Rotate the g.t. 1's pose by the relative rotation
    // first: convert to Eigen::Quaterniond
    Eigen::Quaterniond q_gt1(gt_quat1[0], gt_quat1[1], gt_quat1[2], gt_quat1[3]);
    Eigen::Quaterniond q_rel(qvec_init[0], qvec_init[1], qvec_init[2], qvec_init[3]);
    // Perform the rotation (quaternion multiplication)
    Eigen::Quaterniond q_rotated = q_gt1*q_rel;
    // Convert back to Eigen::Vector4d
    Eigen::Vector4d q_rotated_vec(q_rotated.w(), q_rotated.x(), q_rotated.y(), q_rotated.z());

    // get g.t. relative trans after scaling
    Eigen::Vector3d gt_rel;
    RelativeTransFromGT(gt_quat1, gt_trans1, gt_trans2, gt_rel);
    // get relative rotation
    Eigen::Matrix3d rot_rel = colmap::QuaternionToRotationMatrix(qvec_init);

    // rescale the estimated trans vector
    double scale = gt_rel.norm()/tvec_init.norm();
    std::cout << "scale of relative trans is: " << scale << std::endl;

    // get estimated trans2
    Eigen::Vector3d trans2_est = rot_rel*gt_trans1 + scale*tvec_init;

    // set the 2nd frame's pose
    cmp_image2.SetQvec(q_rotated_vec);
    cmp_image2.SetTvec(trans2_est);

    // check the relative motion via essential mat
    std::cout << "relative rotation of the first pair is: " << std::endl;
    std::cout << qvec_init << std::endl;
    std::cout << "relative translation of the first pair is: " << std::endl;
    std::cout << tvec_init << std::endl;
    std::cout << "relative translation after scaling is: " << std::endl;
    std::cout << trans2_est << std::endl;

    // check the g.t. 2's pose
    std::cout << "rotation of the 2nd g.t. is: " << std::endl;
    std::cout << gt_quat2 << std::endl;
    std::cout << "rotation after apply rel rot is: " << std::endl;
    std::cout << q_rotated_vec << std::endl;

    // start triangulation
    Eigen::Matrix3d calibration = camera.CalibrationMatrix();
    std::cout << "check calibration matrix for the first pair: " << std::endl;
    std::cout << calibration << std::endl;

    // here we do ImageToCamera (in COLMAP it named as ImageToWorld)
    // i.e. normalize the pixel coordinates inside the implementation of TriangulateImage
    std::vector<Eigen::Vector3d> triangulate_3d;
    TriangulateImage(cmp_image1, cmp_image2, camera, matched_vec1, matched_vec2,
                    triangulate_3d);

    // triangulate_3d should has identical length of matched_vec and inlier_mask
    // we skip the outlier by checking inlier_mask and chierality
    int curr_3d_len = 0;
    int inlier_img0 = 0; // debugging inliers of reprojection
    int inlier_img1 = 0;

    double min_ang = colmap::DegToRad(16.0); // follow colmap's default value 
    int inlier_cnt = 0;
    for (int i = 0; i < triangulate_3d.size(); i++){
        if(inlier_mask_rel[i] == 0)
            continue;
        // check the quality of the triangulation
        if(!CheckTriangulateQuality(cmp_image1.ProjectionMatrix(),
                                    cmp_image2.ProjectionMatrix(),
                                    cmp_image1.ProjectionCenter(),
                                    cmp_image2.ProjectionCenter(),
                                    triangulate_3d[i], min_ang)){
            continue;
        }

        inlier_cnt++;
        int orig_idx1 = vec2d1_idx_map[i]; // identical to colmap_vec and sift_vec
        int orig_idx2 = vec2d2_idx_map[i];
        
        // all 3d points are new; i consistent with new_3d_id (0-indexed)
        int new_3d_id = curr_3d_len;
        curr_3d_len++;
        colmap::Point3D new_3d;
        new_3d.SetXYZ(triangulate_3d[i]);
        new_3d.Track().AddElement(0, orig_idx1); // (image_id, 2d_id)
        new_3d.Track().AddElement(1, orig_idx2);

        // DEBUGGING: reprojection errors
        Eigen::Vector2d curr_2d_from0 = key_vec1[orig_idx1];
        // here 2d point is the image pixel/feature point
        double repro_1 = colmap::CalculateSquaredReprojectionError(curr_2d_from0,
                                                                   triangulate_3d[i],
                                                                   cmp_image1.Qvec(),
                                                                   cmp_image1.Tvec(),
                                                                   camera);
        if(repro_1 <= 0.8)
            inlier_img0++;

        Eigen::Vector2d curr_2d_from1 = key_vec2[orig_idx2];
        
        double repro_2 = colmap::CalculateSquaredReprojectionError(curr_2d_from1,
                                                                   triangulate_3d[i],
                                                                   cmp_image2.Qvec(),
                                                                   cmp_image2.Tvec(),
                                                                   camera);
        if(repro_2 <= 0.8)
            inlier_img1++;

        std::cout << "reprojection of 3d point " << new_3d_id << " on image " << 0 << " is "
                  << repro_1 << std::endl;
        std::cout << "reprojection of 3d point " << new_3d_id << " on image " << 1 << " is "
                  << repro_2 << std::endl;
        // DEBUGGING: triangulated points
        if(i%100 == 0){
            std::cout << "ith results: " << std::endl;
            std::cout << "triangulated points: " << std::endl;
            std::cout << triangulate_3d[i] << std::endl;
            std::cout << "image 1's pixel: " << std::endl;
            std::cout << curr_2d_from0 << std::endl;
            std::cout << "image 2's pixel: " << std::endl;
            std::cout << curr_2d_from1 << std::endl;
        }
        global_3d_map[new_3d_id] = new_3d;
        cmp_image1.SetPoint3DForPoint2D(orig_idx1,new_3d_id);
        cmp_image2.SetPoint3DForPoint2D(orig_idx2,new_3d_id);    
    }

    std::cout << "rate of points with reprojection < 0.8 on image 0 is: "
    << (float)inlier_img0/curr_3d_len << std::endl;
    std::cout << "rate of points with reprojection < 0.8 on image 1 is: "
    << (float)inlier_img1/curr_3d_len << std::endl;

    global_image_map[0] = cmp_image1;
    global_image_map[1] = cmp_image2;
    global_keypts_map[0] = key_points1;
    global_keypts_map[1] = key_points2;
    std::cout << "num of 3d points from first pair is: " << 
           global_3d_map.size() << std::endl;
    std::cout << "num of 3d points from first pair inlier mask is: " << 
           inlier_cnt << std::endl;
}