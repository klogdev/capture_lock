#include <iostream>
#include <fstream>
#include <sstream>

#include <vector>
#include <string>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "file_reader/tum_rgbd.h"

void GetSortedFiles(const boost::filesystem::path& directory, std::vector<boost::filesystem::path>& files) {
    // Check if the directory exists and is indeed a directory
    if (!boost::filesystem::exists(directory) || !boost::filesystem::is_directory(directory)) {
        std::cerr << "Provided path is not a directory or does not exist." << std::endl;
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

void OnePairDepthRGB(std::string& rgb_file, std::string& depth_file,
                     std::vector<Eigen::Vector3d>& camera_pts, TUMIntrinsic& paras) {
    cv::Mat rgb_image = cv::imread(rgb_file, cv::IMREAD_COLOR);
    cv::Mat depth_image = cv::imread(depth_file, cv::IMREAD_ANYDEPTH);

    // Check if images are loaded
    if (rgb_image.empty() || depth_image.empty()) {
        std::cerr << "Error loading images" << std::endl;
    }

    for (int v = 0; v < depth_image.rows; v++) {
        for (int u = 0; u < depth_image.cols; u++) {
            uint16_t depth_value = depth_image.at<uint16_t>(v, u);  // Assuming depth image is 16-bit
            if (depth_value > 0) {  // Check for valid depth
                float depth_in_meters = depth_value / paras.scale;  // Convert to meters if necessary
                Eigen::Vector3d curr_pt;
                DepthToCameraSpace(u, v, depth_value, curr_pt, paras);
                camera_pts.push_back(curr_pt)
            }
        }
    }
}

void LoadTUMPoses(const std::string& gt_file, std::vector<Eigen::Vector4d>& quats,
                  std::vector<Eigen::Vector3d>& trans) {
    std::ifstream file(gt_file);
    std::string line;
    double timestamp;

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << gt_file << std::endl;
        return;  // Make sure to exit if file cannot be opened
    }

    while (getline(file, line)) {
        std::istringstream iss(line);
        Eigen::Vector4d curr_quat;
        Eigen::Vector3d curr_trans;

        // Parsing directly into vector elements
        if (!(iss >> timestamp >> curr_quat[0] >> curr_quat[1] >> curr_quat[2] >> curr_quat[3]
                  >> curr_trans[0] >> curr_trans[1] >> curr_trans[2])) {
            std::cerr << "Failed to parse line: " << line << std::endl;
            continue;  // Skip malformed lines or handle error appropriately
        }

        quats.push_back(curr_quat);
        trans.push_back(curr_trans);
    }

    file.close();
}
