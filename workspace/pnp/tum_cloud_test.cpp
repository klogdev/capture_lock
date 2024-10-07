#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/filesystem.hpp>

#include "file_reader/tum_rgbd.h"
#include "util_/file_save.h"


int main(int argc, char** argv) {
    std::string image_dir = argv[1];
    std::string depth_dir = argv[2]; // parental directory for depth maps
    std::string gt_dir = argv[3];

    std::vector<boost::filesystem::path> depth_files;
    std::vector<boost::filesystem::path> image_files;
    DepthRGBMap(image_dir, depth_dir, image_files, depth_files);

    std::vector<std::string> depth_strings;
    for (const auto& path : depth_files) {
        depth_strings.push_back(path.string());  // Convert path to string and add to the list
    }

    std::vector<std::string> image_strings;
    for (const auto& path : image_files) {
        image_strings.push_back(path.string());  // Convert path to string and add to the list
    }
    
    TUMIntrinsic tum_para = TUMIntrinsic();
    std::vector<std::vector<Eigen::Vector2d>> points2D;
    std::vector<std::vector<Eigen::Vector3d>> points3D;
    std::vector<Eigen::Matrix<double, 3, 4>> extrinsic;
    ProcessAllPairs(image_strings, depth_strings, gt_dir, tum_para, points2D, points3D, extrinsic);

    Save3DPoints("/tmp3/Pose_PnP/points3D.txt", points3D);

    return 0;
}
