#include <iostream>
#include <fstream>
#include <sstream>

#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> // For imread

#include "base/pose.h"
#include "base/projection.h"
#include "base/image.h"
#include "base/point2d.h"
#include "base/point3d.h"
#include "base/camera.h"

#include "cost_fxn.h"

#include <ceres/problem.h>

#include "estimate/relative_pose.h"

#include "util_/sift_colmap.h"
#include "feature/sift.h"
#include "feature/image_sift.h"
#include "file_reader/tum_rgbd.h"
#include "file_reader/data_types.h"

#include "test/tum_rgbd_test.h"

void GetSortedFiles(const boost::filesystem::path& directory, std::vector<boost::filesystem::path>& files) {
    // Check if the directory exists and is indeed a directory
    if (!boost::filesystem::exists(directory) || !boost::filesystem::is_directory(directory)) {
        std::cerr << "Provided path: " << directory << " is not a directory or does not exist." << std::endl;
        return;
    }

    // Collect all files in the specified directory
    boost::filesystem::directory_iterator end_itr; // Default construction yields past-the-end
    for (boost::filesystem::directory_iterator itr(directory); itr != end_itr; ++itr) {
        if (boost::filesystem::is_regular_file(itr->status())) {  // Use the status() method to check file type
            files.push_back(itr->path());  // Store the entire path
        }
    }

    // Sort files based on their names (assumed to include sortable timestamps)
    std::sort(files.begin(), files.end(), [](const boost::filesystem::path& a, const boost::filesystem::path& b) {
        return a.filename() < b.filename();  // Compare filenames directly
    });
}

void DepthRGBMap(const boost::filesystem::path& rgb_path,
                 const boost::filesystem::path& depth_path,
                 std::vector<boost::filesystem::path>& all_rgb,
                 std::vector<boost::filesystem::path>& all_depth) {
    GetSortedFiles(rgb_path, all_rgb);
    GetSortedFiles(depth_path, all_depth);

    if (all_rgb.size() != all_depth.size()) {
        std::cerr << "Error: The number of RGB and Depth files does not match." << std::endl;
    }
}

void DepthToCameraSpace(int u, int v, float depth, Eigen::Vector3d& point,
                        TUMIntrinsic& paras) {
    point[2] = depth;  // Depth is the Z value
    point[0] = (u - paras.cx) * depth / paras.fx;  // X value
    point[1] = (v - paras.cy) * depth / paras.fy;  // Y value
}

void OnePairDepthRGB(const std::string& image_file, const std::string& depth_file, 
                     const int curr_idx,
                     std::vector<Eigen::Vector3d>& camera_pts, 
                     std::vector<Eigen::Vector2d>& normalized_pts, 
                     std::unordered_map<int, std::vector<sift::Keypoint>>& global_keypts_map,
                     TUMIntrinsic& paras) {
    cv::Mat rgb_image = cv::imread(image_file, cv::IMREAD_COLOR);
    cv::Mat depth_image = cv::imread(depth_file, cv::IMREAD_ANYDEPTH);

    // Check if images are loaded
    if (rgb_image.empty() || depth_image.empty()) {
        std::cerr << "Error loading image or depth" << std::endl;
    }

    // we only store keypoints and process the camera points
    // in this function, the 3D points processing will be done separately
    Image new_image(image_file, 640, 480);
    std::vector<sift::Keypoint> keypoints = GetKeyPoints(new_image);
    std::vector<sift::Keypoint> selected_key;

    for (int i = 0; i < keypoints.size(); i++) {
        int u = static_cast<int>(keypoints[i].i); // x-coordinate in RGB image
        int v = static_cast<int>(keypoints[i].j); // y-coordinate in RGB image
        
        uint16_t depth_value = depth_image.at<uint16_t>(v, u);  // Assuming depth image is 16-bit
        if (depth_value > 0) {  // Check for valid depth
            double depth_in_meters = depth_value / paras.scale;  // Convert to meters if necessary
            Eigen::Vector3d curr_pt;
            DepthToCameraSpace(u, v, depth_in_meters, curr_pt, paras);
            camera_pts.push_back(curr_pt);
            // equivalent to K^-1*(u, v, 1)
            normalized_pts.push_back(Eigen::Vector2d(curr_pt.x()/depth_in_meters, curr_pt.y()/depth_in_meters));
            selected_key.push_back(keypoints[i]);
        }
    }
    global_keypts_map[curr_idx] = selected_key;
}

void LoadTUMPoses(const std::string& gt_file, std::vector<Eigen::Vector4d>& quats,
                  std::vector<Eigen::Vector3d>& trans) {
    std::ifstream file(gt_file);
    std::string line;
    double timestamp;

    while (getline(file, line)) {
        std::istringstream iss(line);
        Eigen::Vector4d curr_quat;
        Eigen::Vector3d curr_trans;

        // Parsing directly into vector elements
        if (!(iss >> timestamp >> curr_trans[0] >> curr_trans[1] >> curr_trans[2] >> curr_quat[0]
                  >> curr_quat[1] >> curr_quat[2] >> curr_quat[3])) {
            std::cerr << "Failed to parse line: " << line << std::endl;
            continue;  // Skip malformed lines or handle error appropriately
        }

        quats.push_back(curr_quat);
        trans.push_back(curr_trans);
    }

    file.close();
}

void PairsCameraToWorld(const std::vector<Eigen::Vector3d>& camera_pts,
                        const Eigen::Vector4d& quat,
                        const Eigen::Vector3d& trans,
                        std::vector<Eigen::Vector3d>& world_pts) {
    Eigen::Matrix3d R = colmap::QuaternionToRotationMatrix(quat);

    for (const auto& pt : camera_pts) {
        Eigen::Vector3d world_pt = R.transpose()*pt + trans; // R.transpose()*trans;
        // Eigen::Vector3d world_pt = R*pt + R*trans;

        world_pts.push_back(world_pt);
    }
}

void ProcessAllPairs(const std::vector<std::string>& image_files,
                     const std::vector<std::string>& depth_files,
                     const std::string& gt_pose,
                     TUMIntrinsic& paras,
                     std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                     std::vector<std::vector<Eigen::Vector3d>>& points3D,
                     std::vector<Eigen::Matrix<double, 3, 4>>& composed_extrinsic) {
    // Ensure the input lists are of the same length
    assert(depth_files.size() == quats.size());

    std::vector<Eigen::Vector4d> quats;
    std::vector<Eigen::Vector3d> trans;

    colmap::Camera virtual_cam;
    SetVirtualColmapCamera(virtual_cam);
    std::unordered_map<int, colmap::Image*> tum_image_map;
    std::unordered_map<int, colmap::Point3D> tum_3d_map;
    std::unordered_map<int, std::vector<sift::Keypoint>> global_keypts_map;
    int global_3d_id = 0;

    LoadTUMPoses(gt_pose, quats, trans);

    // Process each pair
    for (size_t i = 0; i < std::min(static_cast<size_t>(200), image_files.size()); i++) {
        std::vector<Eigen::Vector2d> normalized_pts;
        std::vector<Eigen::Vector3d> camera_pts;
        std::vector<Eigen::Vector3d> world_pts;

        // Convert depth and RGB to camera space points
        OnePairDepthRGB(image_files[i], depth_files[i], i, camera_pts, normalized_pts, 
                        global_keypts_map, paras);

        // Transform camera space points to world space using GT poses
        PairsCameraToWorld(camera_pts, quats[i], trans[i], world_pts);

        // set current colmap image
        colmap::Image* curr_image = new colmap::Image(SIFTtoCOLMAPImage(i, normalized_pts, virtual_cam));
        Eigen::Matrix3d curr_rot = colmap::QuaternionToRotationMatrix(quats[i]);
        // curr_image->SetQvec(colmap::RotationMatrixToQuaternion(curr_rot.transpose()));
        curr_image->SetQvec(quats[i]);
        curr_image->SetTvec(-curr_rot*trans[i]);

        tum_image_map[i] = curr_image;
        
        colmap::Image* last_image = (i > 0) ? tum_image_map[i - 1] : nullptr;
        
        // world_pts: 3d points from current depth map
        SetPoint3dOneImage(curr_image, last_image, virtual_cam,
                           world_pts, tum_3d_map, 
                           global_keypts_map, global_3d_id);
    }

    for(auto& [id, pt_3d]: tum_3d_map) {
        if(pt_3d.Track().Length() > 1) {
            pt_3d.SetXYZ(pt_3d.XYZ() / pt_3d.Track().Length());
        }
    }

    for(auto& [id, image]: tum_image_map) {
        const Eigen::Vector4d qvec = image->Qvec();  // Camera rotation (quaternion)
        const Eigen::Vector3d tvec = image->Tvec();  // Camera translation

        for (int i = 0; i < image->Points2D().size(); i++) {
            colmap::Point2D& point2D = image->Point2D(i);
            if (!point2D.HasPoint3D()) continue;

            const int point3D_id = point2D.Point3DId();
            colmap::Point3D& point3D = tum_3d_map.at(point3D_id);
            double curr_res = colmap::CalculateSquaredReprojectionError(point2D.XY(),
                                                                        point3D.XYZ(),
                                                                        qvec, tvec, virtual_cam);

            if(curr_res == std::numeric_limits<double>::max()) {
                point2D.SetPoint3DId(colmap::kInvalidPoint3DId);
                point3D.Track().DeleteElement(id, i);
            }
        }
    }

    TUMBundle(tum_image_map, tum_3d_map, virtual_cam);

    // CheckTUMResidual(tum_image_map[0], virtual_cam, tum_3d_map);

    for(auto& [img_id, value]: tum_image_map) {
        std::vector<Eigen::Vector3d> curr_3d;
        std::vector<Eigen::Vector2d> curr_2d;
        RetrievePairsfromImage(value, tum_3d_map, curr_2d, curr_3d);
        if(curr_3d.size() < 4 || curr_2d.size() < 4) continue;
        points3D.push_back(curr_3d);
        points2D.push_back(curr_2d);
        composed_extrinsic.push_back(colmap::ComposeProjectionMatrix(value->Qvec(), value->Tvec()));
    }
}

void SetVirtualColmapCamera(colmap::Camera& virtual_camera) {
    std::vector<double> intrinsic = {1, 0, 0}; // follow colmap's order f, cx, cy

    virtual_camera.SetCameraId(1); // only one camera
    virtual_camera.SetModelId(colmap::SimplePinholeCameraModel::model_id);
    virtual_camera.SetParams(intrinsic);
}

void SetPoint3dOneImage(colmap::Image* curr_img,
                        colmap::Image* last_img,  
                        colmap::Camera& camera,
                        std::vector<Eigen::Vector3d>& point_3d,
                        std::unordered_map<int, colmap::Point3D>& global_3d_map,
                        std::unordered_map<int, std::vector<sift::Keypoint>>& global_keypts_map,
                        int& curr_3d_idx) {

    std::map<int, std::pair<int, char>> match_idx;
    std::vector<char> inlier_mask;

    if (last_img != nullptr) {  // Check if there's a valid last image
        std::vector<sift::Keypoint> last_key_points = global_keypts_map[last_img->ImageId()];
        std::vector<sift::Keypoint> curr_key_points = global_keypts_map[curr_img->ImageId()];
        auto matches = sift::find_keypoint_matches(last_key_points, curr_key_points);

        std::vector<Eigen::Vector2d> matched1, matched2;
        for (const auto& m : matches) {
            if (m.second < point_3d.size() && m.first < point_3d.size()) {
                match_idx[m.second] = std::make_pair(m.first, '0'); // map current images' idx to the last
                matched1.push_back(last_img->Point2D(m.first).XY());
                matched2.push_back(curr_img->Point2D(m.second).XY());
            }
        }

        // start relative pose estimation w/ RANSAC for a pre-filtering
        colmap::RANSACOptions ransac_options = colmap::RANSACOptions();
        ransac_options.max_error = 0.05;
        Eigen::Vector4d qvec_rel = Eigen::Vector4d(0, 0, 0, 1); // init relative pose
        Eigen::Vector3d tvec_rel = Eigen::Vector3d::Zero();     // w/ 0 rot and trans
        std::vector<Eigen::Vector3d> point_rel;
        // use customized relative pose estimator w/ inlier masks
        size_t num_inliers = 
            RelativePoseWMask(ransac_options, camera, matched1, matched2, 
                                        &qvec_rel, &tvec_rel, &inlier_mask, &point_rel);
    }

    int cnt = 0;
    if(match_idx.size() != 0) {
        for(auto& m : match_idx) {
        if (cnt < inlier_mask.size()) {
            m.second.second = inlier_mask[cnt];  // Update the value at the current key in the map
            cnt++;
        } else {
                break; // Avoid going out of bounds if inlier_mask has fewer elements than match_idx
            }
        }
    }

    for (int i = 0; i < point_3d.size(); i++) {
        if (match_idx.empty() || match_idx.find(i) == match_idx.end()) {
            curr_img->SetPoint3DForPoint2D(i, curr_3d_idx);
            colmap::Point3D new_3d;
            new_3d.SetXYZ(point_3d[i]);
            new_3d.Track().AddElement(curr_img->ImageId(), i);
            global_3d_map[curr_3d_idx] = new_3d;
            curr_3d_idx++;
        } else {
            int idx_last = match_idx[i].first;
            if(match_idx[i].second == '0') continue;

            if (last_img && idx_last >= last_img->NumPoints2D()) {
                continue;  // Avoid out-of-bounds access
            }
            int last_3d_id = last_img->Point2D(idx_last).Point3DId();
            if (global_3d_map.find(last_3d_id) == global_3d_map.end()) {
                continue; // Ensure we have a valid 3D point reference
            }

            colmap::Point3D& old_point3d = global_3d_map[last_3d_id];
            // std::cout << "last frame's 3d: " << (old_point3d.XYZ()/old_point3d.Track().Length()).transpose() << std::endl; 
            // std::cout << "curr frame's 3d: " << point_3d[i].transpose() << std::endl; 

            double norm = (point_3d[i] - (old_point3d.XYZ()/old_point3d.Track().Length())).norm();
            if(norm > 1.0) {
                // std::cout << "point w/ " << norm << " and " <<  point_3d[i].transpose() << " rejected" << std::endl;
                continue;
            }
            old_point3d.SetXYZ((point_3d[i] + old_point3d.XYZ()));
            old_point3d.Track().AddElement(curr_img->ImageId(), i);
            curr_img->SetPoint3DForPoint2D(i, last_3d_id);
        }
    }
}

void TUMBundle(std::unordered_map<int, colmap::Image*>& global_img_map,
               std::unordered_map<int, colmap::Point3D>& global_3d_map,
               colmap::Camera& camera, double anchor_weight) {
    ceres::Problem problem;

    std::cout << "we have num of 3d points: " << global_3d_map.size() << std::endl;
    for (auto& [point3D_id, point3D] : global_3d_map) {
        if(point3D.Track().Length() < 5) continue;
        std::cout << "curr covisible is " << point3D.Track().Length() << std::endl;
        // Add 3D point as a parameter block
        problem.AddParameterBlock(point3D.XYZ().data(), 3);

        // Add anchor residual for 3D point stabilization
        ceres::CostFunction* anchor_cost_function = 
            AnchorCostFxn::Create(point3D.XYZ(), anchor_weight);
        problem.AddResidualBlock(anchor_cost_function, nullptr, point3D.XYZ().data());
    }

    int num_residuals = 0;
    int num_parameters = 0;
    int num_blocks = 0;

    int residual_count = 0;
    for (auto& [image_id, image] : global_img_map) {
        const Eigen::Vector4d qvec = image->Qvec();  // Camera rotation (quaternion)
        const Eigen::Vector3d tvec = image->Tvec();  // Camera translation

        for (const auto& point2D : image->Points2D()) {
            if (!point2D.HasPoint3D()) continue;

            const int point3D_id = point2D.Point3DId();
            colmap::Point3D& point3D = global_3d_map.at(point3D_id);
            if(point3D.Track().Length() < 5) continue;

            ceres::CostFunction* cost_function = BAConstPoseCostFxn<colmap::SimplePinholeCameraModel>::Create(
                qvec, tvec, point2D.XY());

            problem.AddResidualBlock(cost_function, nullptr, point3D.XYZ().data(), camera.ParamsData());
            residual_count += 2; // Each observation has two residuals (x and y)
            num_residuals += cost_function->num_residuals();
            for (int i = 0; i < cost_function->parameter_block_sizes().size(); ++i) {
                num_parameters += cost_function->parameter_block_sizes()[i];
            }
            
            // Confirm adding each residual block
            // std::cout << "Added residual for 2D-3D correspondence in image ID: " << image_id
            //           << ", 2D point index: " << &point2D - &image->Points2D()[0]
            //           << ", associated 3D point ID: " << point3D_id << std::endl;

            num_blocks += problem.ParameterBlockSize(point3D.XYZ().data());
        }
    }

    std::cout << "Total residuals added: " << residual_count << std::endl;
    std::cout << "Total params added: " << num_blocks << std::endl;

    problem.SetParameterBlockConstant(camera.ParamsData());

    ceres::Solver::Options options;
    // options.minimizer_progress_to_stdout = true;
    // options.logging_type = ceres::PER_MINIMIZER_ITERATION;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;  // Can try 'LEVENBERG_MARQUARDT' if needed

    // std::cout << "Residuals before optimization:" << std::endl;
    // PrintLargeResiduals(problem);

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    ceres::CRSMatrix jacobian;
    problem.Evaluate(ceres::Problem::EvaluateOptions(), nullptr, nullptr, nullptr, &jacobian);

    // std::cout << "number of Jacobian col: " << jacobian.cols.size() <<std::endl;
    // std::cout << "number of Jacobian row: " << jacobian.rows.size() <<std::endl;

    // Jacobian size is approximately: num_residuals x num_parameters
    std::cout << "Jacobian size: " << num_residuals << " x " << num_parameters << std::endl;
}

void SetBAOptions(colmap::BundleAdjustmentOptions& ba_options) {
    ba_options.solver_options.num_threads = 1;
    ba_options.loss_function_type = colmap::BundleAdjustmentOptions::LossFunctionType::SOFT_L1;
    ba_options.refine_focal_length = false;
    ba_options.refine_extra_params = false;
    ba_options.refine_extrinsics = false;
}

void RetrievePairsfromImage(colmap::Image* curr_img, 
                            std::unordered_map<int, colmap::Point3D>& global_3d_map,
                            std::vector<Eigen::Vector2d>& point2ds,
                            std::vector<Eigen::Vector3d>& point3ds) {
    for(const colmap::Point2D& p: curr_img->Points2D()) {
        if (!p.HasPoint3D()) continue;

        colmap::point3D_t global_3d_key = p.Point3DId();
        if(global_3d_map[global_3d_key].Track().Length() < 5) continue;

        Eigen::Vector3d from2d_to3d = global_3d_map.at(global_3d_key).XYZ();
        
        // Check for NaN, Inf, or extreme values
        if (!from2d_to3d.allFinite()) {
            std::cerr << "Warning: Non-finite 3D coordinates detected for point ID " << global_3d_key << std::endl;
            continue; // Skip this point
        }

        if (from2d_to3d.norm() > 1e5) { // Example threshold
            std::cerr << "Warning: 3D point with excessively large norm detected for point ID " << global_3d_key << std::endl;
            continue; // Skip this point
        }

        // Optionally check individual components against expected bounds
        // if (abs(from2d_to3d.x()) > max_allowed_x || ...) { ... }

        point2ds.push_back(p.XY());
        point3ds.push_back(from2d_to3d);
    }
}
