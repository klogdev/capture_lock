#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <Eigen/Core> 
#include <algorithm>

#include "base/pose.h"
#include "util_/file_save.h"
#include "file_reader/tum_rgbd.h"

void LoadOrbLoaded(const std::string& filename,
                   std::vector<std::vector<Eigen::Vector2d>>& points2D,
                   std::vector<std::vector<Eigen::Vector3d>>& points3D,
                   std::vector<Eigen::Matrix3x4d>& composed_extrinsic,
                   int seq_num=1) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    TUMIntrinsic intri = TUMIntrinsic(seq_num);

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;  // Skip blank lines that separate blocks

        // Check if line starts with "# Pose: "
        if (line.rfind("# Pose: ", 0) == 0) {
            // Remove the "# Pose: " prefix and parse the rest of the pose data
            line = line.substr(8);  // Skip the prefix
            std::istringstream pose_stream(line);
            double tx, ty, tz, qx, qy, qz, qw;
            if (!(pose_stream >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
                std::cerr << "Error: Unable to read pose line. Skipping." << std::endl;
                continue;
            }

            // Convert quaternion and translation to extrinsics matrix
            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Eigen::Matrix3x4d extrinsics;
            Eigen::Matrix3d rot = quat.toRotationMatrix().transpose();
            extrinsics.block<3, 3>(0, 0) = rot;
            extrinsics.block<3, 1>(0, 3) = -rot*Eigen::Vector3d(tx, ty, tz);
            composed_extrinsic.push_back(extrinsics);

            // Prepare to store 2D-3D pairs for this image
            std::vector<Eigen::Vector2d> image_points2D;
            std::vector<Eigen::Vector3d> image_points3D;

            // Read 2D-3D points for the current image until a blank line
            while (std::getline(file, line)) {
                line = Trim(line);  
                if (line.empty()) break;  // Exit loop if a blank line is found

                std::istringstream point_stream(line);
                double u, v, x, y, z;
                if (!(point_stream >> u >> v >> x >> y >> z)) {
                    std::cerr << "Warning: Malformed 2D-3D point line. Skipping." << std::endl;
                    continue;
                }
                image_points2D.push_back(Eigen::Vector2d((u - intri.cx)/intri.fx, (v - intri.cy)/intri.fy));
                image_points3D.push_back(Eigen::Vector3d(x, y, z));
            }

            // Store the 2D and 3D points for the current image
            points2D.push_back(image_points2D);
            points3D.push_back(image_points3D);

            std::cout << "Image loaded. Extrinsics: [" << tx << ", " << ty << ", " << tz 
                      << "; " << qx << ", " << qy << ", " << qz << ", " << qw << "]"
                      << " with " << image_points2D.size() << " 2D-3D pairs." << std::endl;
        }
    }

    std::cout << "Total images loaded: " << composed_extrinsic.size() << std::endl;
    std::cout << "Total 2D points sets: " << points2D.size() << std::endl;
    std::cout << "Total 3D points sets: " << points3D.size() << std::endl;
    file.close();
}