#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <Eigen/Dense> 

#include "base/pose.h"

// Function to load data into vectors
void LoadColmapPairs(const std::string& filename,
                     std::vector<std::vector<Eigen::Vector2d>>& points2D,
                     std::vector<std::vector<Eigen::Vector3d>>& points3D,
                     std::vector<Eigen::Matrix3x4d>& composed_extrinsic) {
    // std::cout << "no seg fault when call loader" << std::endl;
    std::ifstream file(filename);
    std::string line;

    // std::cout << "no seg fault after stream" << std::endl;

    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    while (getline(file, line)) {
        if (line.rfind("Image ID:", 0) == 0) {
            // Start a new set of points and extrinsic for this image
            std::vector<Eigen::Vector2d> curr_2d;
            std::vector<Eigen::Vector3d> curr_3d;
            Eigen::Matrix3x4d extrinsic;

            // Read the pose line
            getline(file, line);
            std::istringstream pose_iss(line);
            std::string label;
            double qw, qx, qy, qz, tx, ty, tz;
            pose_iss >> label >> label >> qw >> qx >> qy >> qz >> tx >> ty >> tz;

            // Convert quaternion to rotation matrix and create the extrinsic matrix
            Eigen::Vector4d q(qw, qx, qy, qz);
            Eigen::Matrix3d rotation = colmap::QuaternionToRotationMatrix(q);
            extrinsic.block<3, 3>(0, 0) = rotation;
            extrinsic.block<3, 1>(0, 3) << tx, ty, tz;

            composed_extrinsic.push_back(extrinsic);

            // Skip the "2D-3D pairs:" line
            getline(file, line);

            // Read the 2D-3D pairs line by line
            while (getline(file, line) && !line.empty()) {
                std::istringstream pair_iss(line);
                double u, v, X, Y, Z;
                char ignore_char;
                pair_iss >> u >> v >> ignore_char >> X >> ignore_char >> Y >> ignore_char >> Z;

                // Store 2D and 3D points
                curr_2d.push_back(Eigen::Vector2d(u, v));
                curr_3d.push_back(Eigen::Vector3d(X, Y, Z));
            }
            points2D.push_back(curr_2d);
            points3D.push_back(curr_3d);
        }
    }

    file.close();
}

// Function to test by printing loaded data
void printData(const std::vector<std::vector<Eigen::Vector2d>>& points2D,
               const std::vector<std::vector<Eigen::Vector3d>>& points3D,
               const std::vector<Eigen::Matrix3x4d>& composed_extrinsic) {
    for (size_t i = 0; i < points2D.size(); i++) {
        std::cout << "Image " << i << " Extrinsic Matrix:\n" << composed_extrinsic[i] << "\n\n";
        
        std::cout << "2D-3D pairs:" << std::endl;
        for (size_t j = 0; j < 5; j++) {
            std::cout << "2D: (" << points2D[i][j].x() << ", " << points2D[i][j].y()
                      << "), 3D: (" << points3D[i][j].x() << ", " << points3D[i][j].y()
                      << ", " << points3D[i][j].z() << ")" << std::endl;
        }
        std::cout << "-------------------------------------" << std::endl;
    }
}
