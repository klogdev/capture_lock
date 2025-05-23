#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <Eigen/Core> 
#include <algorithm>

#include "base/pose.h"
#include "util_/file_save.h"
#include "file_reader/use_geo.h"

void LoadUseGeo(const std::string& filename,
                     std::vector<std::vector<Eigen::Vector2d>>& points2D,
                     std::vector<std::vector<Eigen::Vector3d>>& points3D,
                     std::vector<Eigen::Matrix3x4d>& composed_extrinsic) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;  // Blank line to separate each block

        // Parse the extrinsic pose (qw, qx, qy, qz, tx, ty, tz)
        std::istringstream pose_stream(line);
        double qw, qx, qy, qz, tx, ty, tz;
        if (!(pose_stream >> qx >> qy >> qz >> qw >>tx >> ty >> tz)) {
            std::cerr << "Error: Unable to read pose line. Skipping." << std::endl;
            continue;
        }

        // Convert quaternion and translation to extrinsics matrix
        Eigen::Quaterniond quat(qw, qx, qy, qz);
        Eigen::Matrix3x4d extrinsics;
        extrinsics.block<3, 3>(0, 0) = quat.toRotationMatrix();
        extrinsics.block<3, 1>(0, 3) = Eigen::Vector3d(tx, ty, tz);
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
            image_points2D.push_back(Eigen::Vector2d(u, v));
            image_points3D.push_back(Eigen::Vector3d(x, y, z));
        }

        // Store the 2D and 3D points for the current image
        points2D.push_back(image_points2D);
        points3D.push_back(image_points3D);

        std::cout << "Image loaded. Extrinsics: [" << qw << ", " << qx << ", " << qy
                  << ", " << qz << "; " << tx << ", " << ty << ", " << tz << "]"
                  << " with " << image_points2D.size() << " 2D-3D pairs." << std::endl;
    }

    std::cout << "Total images loaded: " << composed_extrinsic.size() << std::endl;
    std::cout << "Total 2D points sets: " << points2D.size() << std::endl;
    std::cout << "Total 3D points sets: " << points3D.size() << std::endl;
    file.close();
}
