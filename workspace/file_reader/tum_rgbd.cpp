#include <iostream>
#include <fstream>
#include <sstream>

#include <vector>
#include <string>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> // For imread
#include <opencv2/features2d.hpp>

#include "base/pose.h"
#include "base/projection.h"
#include "base/image.h"
#include "base/point2d.h"
#include "base/point3d.h"
#include "base/camera.h"

#include "optim/bundle_adjustment.h"

#include "util_/sift_colmap.h"
#include "file_reader/tum_rgbd.h"
#include "file_reader/data_types.h"

#include "global_bundle.h"

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
                     std::vector<Eigen::Vector3d>& camera_pts, 
                     std::vector<Eigen::Vector2d>& normalized_pts, TUMIntrinsic& paras) {
    cv::Mat rgb_image = cv::imread(image_file, cv::IMREAD_COLOR);
    cv::Mat depth_image = cv::imread(depth_file, cv::IMREAD_ANYDEPTH);

    // Check if images are loaded
    if (rgb_image.empty() || depth_image.empty()) {
        std::cerr << "Error loading images" << std::endl;
    }

    // ORB feature detection
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // Detect ORB keypoints in the RGB image
    orb->detectAndCompute(rgb_image, cv::noArray(), keypoints, descriptors);

    for (const auto& kp : keypoints) {
        int u = static_cast<int>(kp.pt.x); // x-coordinate in RGB image
        int v = static_cast<int>(kp.pt.y); // y-coordinate in RGB image
        
        uint16_t depth_value = depth_image.at<uint16_t>(v, u);  // Assuming depth image is 16-bit
        if (depth_value > 0) {  // Check for valid depth
            double depth_in_meters = depth_value / paras.scale;  // Convert to meters if necessary
            Eigen::Vector3d curr_pt;
            DepthToCameraSpace(u, v, depth_in_meters, curr_pt, paras);
            camera_pts.push_back(curr_pt);
            // equivalent to K^-1*(u, v, 1)
            normalized_pts.push_back(Eigen::Vector2d(curr_pt.x()/depth_in_meters, curr_pt.y()/depth_in_meters));
        }
    }
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
        std::cout << "current loaded rot" << std::endl;
        std::cout << curr_quat << std::endl;
    }

    file.close();
}

void PairsCameraToWorld(const std::vector<Eigen::Vector3d>& camera_pts,
                        const Eigen::Vector4d& quat,
                        const Eigen::Vector3d& trans,
                        std::vector<Eigen::Vector3d>& world_pts) {
    Eigen::Matrix3d R = colmap::QuaternionToRotationMatrix(quat);

    for (const auto& pt : camera_pts) {
        Eigen::Vector3d world_pt = R.transpose() * pt - R.transpose() * trans;
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
    std::unordered_map<int, colmap::Image> tum_image_map;
    std::unordered_map<int, colmap::Point3D> tum_3d_map;
    int global_3d_id = 0;

    LoadTUMPoses(gt_pose, quats, trans);

    // Process each pair
    for (size_t i = 0; i < image_files.size(); i++) {
        std::vector<Eigen::Vector2d> normalized_pts;
        std::vector<Eigen::Vector3d> camera_pts;
        std::vector<Eigen::Vector3d> world_pts;

        // Convert depth and RGB to camera space points
        OnePairDepthRGB(image_files[i], depth_files[i], camera_pts, normalized_pts, paras);

        // Transform camera space points to world space using GT poses
        PairsCameraToWorld(camera_pts, quats[i], trans[i], world_pts);

        // set current colmap image
        colmap::Image curr_image = SIFTtoCOLMAPImage(i, normalized_pts, virtual_cam);
        tum_image_map[i] = curr_image;
        SetPoint3dOneImage(curr_image, world_pts, tum_3d_map, global_3d_id);
        // set the 2d normalized pts and g.t. poses as usual
        points2D.push_back(normalized_pts);
        composed_extrinsic.push_back(colmap::ComposeProjectionMatrix(quats[i], trans[i]));
    }

    colmap::BundleAdjustmentOptions ba_options;
    SetBAOptions(ba_options); // set extrinsic & intrinsic as constant

    std::vector<int> global_image_opt;
    for(const auto& [img_id, value]: tum_image_map){
        global_image_opt.push_back(img_id);
    }
    std::vector<int> global_const_pose;
    bool run_ba = GlobalBundleAdjuster(ba_options, virtual_cam, tum_image_map,
                                       tum_3d_map, global_image_opt, 
                                       global_const_pose, Dataset::Any);  

    for(auto& [img_id, value]: tum_image_map) {
        std::vector<Eigen::Vector3d> curr_3d;
        Retrieve3DfromImage(value, tum_3d_map, curr_3d);
        points3D.push_back(curr_3d);
    }
}

void SetVirtualColmapCamera(colmap::Camera& virtual_camera) {
    std::vector<double> intrinsic = {1, 1, 0, 0}; // follow colmap's order fx, fy, cx, cy

    virtual_camera.SetCameraId(1); // only one camera
    virtual_camera.SetModelId(colmap::SimplePinholeCameraModel::model_id);
    virtual_camera.SetParams(intrinsic);
}

void SetPoint3dOneImage(colmap::Image& curr_img, std::vector<Eigen::Vector3d>& point_3d,
                        std::unordered_map<int, colmap::Point3D>& global_3d_map,
                        int& curr_3d_idx) {
    // assume 3d points has identical size as 2d that registered in colmap::Image
    for(int i = 0; i < point_3d.size(); i++) {
        curr_img.SetPoint3DForPoint2D(i, curr_3d_idx);
        colmap::Point3D new_3d;
        new_3d.SetXYZ(point_3d[i]);
        new_3d.Track().AddElement(curr_img.ImageId(), i);
        global_3d_map[curr_3d_idx] = new_3d;
        curr_3d_idx++;
    }
}

void SetBAOptions(colmap::BundleAdjustmentOptions& ba_options) {
    ba_options.solver_options.num_threads = 1;
    ba_options.loss_function_type = colmap::BundleAdjustmentOptions::LossFunctionType::SOFT_L1;
    ba_options.refine_focal_length = false;
    ba_options.refine_extra_params = false;
    ba_options.refine_extrinsics = false;
}

void Retrieve3DfromImage(colmap::Image& curr_img, 
                         std::unordered_map<int, colmap::Point3D>& global_3d_map,
                         std::vector<Eigen::Vector3d>& point3ds) {
    for(const colmap::Point2D& p: curr_img.Points2D()) {
        colmap::point3D_t global_3d_key = p.Point3DId();
        Eigen::Vector3d from2d_to3d = global_3d_map.at(global_3d_key).XYZ();
        point3ds.push_back(from2d_to3d);
    }
}