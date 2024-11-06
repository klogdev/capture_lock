#include <Eigen/Core>
#include <memory>

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <sstream>
#include <random>
#include <cmath>
#include <limits>

#include "pnp/pnp_helper.h"
#include "util_/math_helper.h"

void GetIntrinsic(Eigen::Matrix3d& k) {
    k << 800, 0, 320,
         0, 800, 240,
         0, 0, 1;
}

void CameraSpaceShift(const std::vector<Eigen::Vector3d>& camera_pts, 
                      const Eigen::Vector3d& trans,
                      std::vector<Eigen::Vector3d>& shifted_pts) {
    shifted_pts.clear();

    // Reserve the necessary space in the destination vector to improve performance
    shifted_pts.reserve(camera_pts.size());

    for (const Eigen::Vector3d& pt : camera_pts) {
        // Add the translation vector to the current point
        shifted_pts.push_back(pt + trans);
    }
}

void GenOneSetNoise2D(std::vector<Eigen::Vector3d>& camera_space_points, 
                      std::vector<Eigen::Vector2d>& one_set_2d,
                      Eigen::Matrix3d& k, double sigma) {
    Eigen::Matrix3d k_inv = k.inverse();

    for(int i = 0; i < camera_space_points.size(); i++) {
        Eigen::Vector3d curr_camera_pt = camera_space_points[i];
        Eigen::Vector3d curr_projected = k*curr_camera_pt;
        double curr_pix_x = curr_projected.x()/curr_projected.z();
        double curr_pix_y = curr_projected.y()/curr_projected.z();
        Eigen::Vector3d noised_image_pt = 
                    Eigen::Vector3d(RandomGaussian(curr_pix_x, sigma),
                                    RandomGaussian(curr_pix_y, sigma),
                                    1);
        Eigen::Vector3d noised_camera_pt = k_inv*noised_image_pt;
        one_set_2d.push_back(Eigen::Vector2d(noised_camera_pt.x()/noised_camera_pt.z(),
                                    noised_camera_pt.y()/noised_camera_pt.z()));
    }
}

void Perturbation3D(std::vector<Eigen::Vector3d>& camera_space_points, double sigma) {
    // Create a random engine and a Gaussian distribution
    std::random_device rd;  // Seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::normal_distribution<> dis(0, sigma); // mean 0 and standard deviation sigma

    // Apply Gaussian noise to each point
    for (Eigen::Vector3d& p : camera_space_points) {
        p.x() += dis(gen);
        p.y() += dis(gen);
        p.z() += dis(gen);
    }
}

void AddOutlier2D(std::vector<Eigen::Vector2d>& points2D, double outlier_rate, 
                  const int image_x, const int image_y, Eigen::Matrix3d& k_inv) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(0, image_x);
    std::uniform_real_distribution<> dis_y(0, image_y);
    
    int num_points = points2D.size();
    int num_outliers = static_cast<int>(num_points * outlier_rate);
    
    for (int i = 0; i < num_outliers; i++) {
        int index = rand() % num_points;
        Eigen::Vector3d norm_point = k_inv*Eigen::Vector3d(dis_x(gen), dis_y(gen), 1); // Random outlier
        points2D[index] = Eigen::Vector2d(norm_point.x(), norm_point.y());
    }
}

//////////////////////////////////////////
// Helper function for EPnP synthetic data
/////////////////////////////////////////
void EPnPBoxCorner(std::vector<Eigen::Vector3d>& camera_space_points) {

    // Define the ranges for x, y, and z
    double x_values[2] = {-2, 2};
    double y_values[2] = {-2, 2};
    double z_values[2] = {4, 8};

    // Generate the corners
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                camera_space_points.push_back(Eigen::Vector3d(x_values[i], y_values[j], z_values[k]));
            }
        }
    }
}

void EPnPPlanar(std::vector<Eigen::Vector3d>& camera_space_points) {
    // Define the ranges for x and y, and set a fixed z value
    double x_values[3] = {-2, 0, 2};  // Using three positions along x
    double y_values[3] = {-2, 0, 2};  // Using three positions along y
    double fixed_z_value = 6;  // Choose a mid-range value between 4 and 8

    // Generate points on the plane
    // Select 8 points including corners, midpoints of edges, and center point
    camera_space_points.push_back(Eigen::Vector3d(x_values[0], y_values[0], fixed_z_value)); // Corner
    camera_space_points.push_back(Eigen::Vector3d(x_values[0], y_values[1], fixed_z_value)); // Edge midpoint
    camera_space_points.push_back(Eigen::Vector3d(x_values[0], y_values[2], fixed_z_value)); // Corner
    camera_space_points.push_back(Eigen::Vector3d(x_values[1], y_values[0], fixed_z_value)); // Edge midpoint
    camera_space_points.push_back(Eigen::Vector3d(x_values[1], y_values[2], fixed_z_value)); // Edge midpoint
    camera_space_points.push_back(Eigen::Vector3d(x_values[2], y_values[0], fixed_z_value)); // Corner
    camera_space_points.push_back(Eigen::Vector3d(x_values[2], y_values[1], fixed_z_value)); // Edge midpoint
    camera_space_points.push_back(Eigen::Vector3d(x_values[2], y_values[2], fixed_z_value)); // Corner
}

void EPnPInsideRand(std::vector<Eigen::Vector3d>& camera_space_points,
                    int num_pts) {
    int count = 0;
    while (count < num_pts) {
        double curr_x = RandomUniform(-2, 2);
        double curr_y = RandomUniform(-2, 2);
        double curr_z = RandomUniform(4, 8);
        Eigen::Vector3d curr_trial = Eigen::Vector3d(curr_x, curr_y, curr_z);
        double u = curr_x*800/curr_z + 320; // default intrinsic from EPnP simulator
        double v = curr_y*800/curr_z + 240;

        if(u < 0 || u > 640 || v < 0 || v > 480)
            continue;
        camera_space_points.push_back(Eigen::Vector3d(curr_x, curr_y, curr_z));
        count++;
    }
}

void EPnPPlanarRand(std::vector<Eigen::Vector3d>& camera_space_points,
                    int num_pts) {
    int count = 0;
    while (count < num_pts) {
        double curr_x = RandomUniform(-2, 2);
        double curr_y = RandomUniform(-2, 2);
        double curr_z = 6;
        Eigen::Vector3d curr_trial = Eigen::Vector3d(curr_x, curr_y, curr_z);
        double u = curr_x*800/curr_z + 320; // default intrinsic from EPnP simulator
        double v = curr_y*800/curr_z + 240;

        if(u < 0 || u > 640 || v < 0 || v > 480)
            continue;
        camera_space_points.push_back(Eigen::Vector3d(curr_x, curr_y, curr_z));
        count++;
    }
}

void EPnPRandomRot(Eigen::Matrix3d& rot) {
    double alpha = RandomUniform(0, 45);
    double beta = RandomUniform(0, 45);
    double gamma = RandomUniform(0, 45);

    alpha = alpha * M_PI / 180.0;
    beta = beta * M_PI / 180.0;
    gamma = gamma * M_PI / 180.0;

    // Rotation matrices for each axis
    Eigen::Matrix3d Rx;
    Rx = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());

    Eigen::Matrix3d Ry;
    Ry = Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY());

    Eigen::Matrix3d Rz;
    Rz = Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ());

    // Combined rotation matrix
    rot = Rz * Ry * Rx;
}

void EPnPRandomTrans(Eigen::Vector3d& trans) {
    double x = RandomUniform(5, 15);
    double y = RandomUniform(5, 15);
    double z = RandomUniform(20, 50);

    trans << x, y, z;
}
