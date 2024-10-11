#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <iostream>
#include <boost/format.hpp>
#include <cmath>  // For acos, M_PI

#include "base/pose.h"

#include "util/types.h"

Eigen::Vector4d GenRandomRot() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distr(-1.0, 1.0);

    Eigen::Vector4d quat;
    double norm;

    do {
        quat = Eigen::Vector4d(distr(gen), distr(gen), distr(gen), distr(gen));
        norm = quat.norm();
    } while (!(0.2 <= norm && norm <= 1.0));

    quat /= norm;

    return quat;
}

double RandomUniform(double x_ini, double x_end) {
    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 gen(rd()); // Seed the generator
    std::uniform_real_distribution<> distr(x_ini, x_end); // Define the range

    double x = distr(gen); // Generate a random number

    return x;
}

double RandomGaussian(double mean, double std) {
    std::random_device rd; // Seed with a real random value, if available
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    
    std::normal_distribution<> distr(mean, std); // Create a normal distribution

    double random_number = distr(gen); // Generate a random number

    return random_number;
}

double RelativeQuatErr(const Eigen::Vector4d& quat_gt, const Eigen::Vector4d& quat_est) {
    double diff = (quat_gt - quat_est).norm();
    double est_norm = quat_est.norm();

    return diff/est_norm;
}

double MaxAxisAlign(const Eigen::Matrix3d& rot_gt, const Eigen::Matrix3d& rot_est) {
    // Extract the column vectors of the rotation matrices
    Eigen::Vector3d X1 = rot_gt.col(0);
    Eigen::Vector3d Y1 = rot_gt.col(1);
    Eigen::Vector3d Z1 = rot_gt.col(2);

    Eigen::Vector3d X2 = rot_est.col(0);
    Eigen::Vector3d Y2 = rot_est.col(1);
    Eigen::Vector3d Z2 = rot_est.col(2);

    // Compute dot products (cosine of angles between corresponding axes)
    double e_x = X1.dot(X2);
    double e_y = Y1.dot(Y2);
    double e_z = Z1.dot(Z2);

    // Clamp the values to avoid acos domain errors (due to floating point inaccuracies)
    e_x = std::max(-1.0, std::min(1.0, e_x));
    e_y = std::max(-1.0, std::min(1.0, e_y));
    e_z = std::max(-1.0, std::min(1.0, e_z));

    // Compute the angles (in radians)
    double angle_x = std::acos(e_x);
    double angle_y = std::acos(e_y);
    double angle_z = std::acos(e_z);

    // Convert angles to degrees
    angle_x *= 180.0 / M_PI;
    angle_y *= 180.0 / M_PI;
    angle_z *= 180.0 / M_PI;

    // Return the maximum absolute angle as the rotation error
    return std::max({std::abs(angle_x), std::abs(angle_y), std::abs(angle_z)});
}

double CosineDifference(const Eigen::Vector4d& quat_gt, const Eigen::Vector4d& quat_est) {
    // Ensure the quaternions are normalized
    Eigen::Vector4d q_gt_norm = quat_gt.normalized();
    Eigen::Vector4d q_est_norm = quat_est.normalized();

    // Compute the dot product between the two quaternions
    double dot_product = std::abs(q_gt_norm.dot(q_est_norm));  // Ensure it's positive to take the shortest path

    // Clamp the dot product to the valid range for acos to avoid numerical issues
    dot_product = std::max(-1.0, std::min(1.0, dot_product));

    // Compute the angular difference in radians
    double angle_diff = 2 * std::acos(dot_product);

    // Convert the angle difference from radians to degrees
    return angle_diff;

}

double RelativeTransErr(const Eigen::Vector3d& gt, const Eigen::Vector3d& estimate) {
    double diff = (gt - estimate).norm();
    double est_norm = estimate.norm();

    return diff/est_norm;
}

void CalculateCoM(const std::vector<Eigen::Vector3d>& point_cloud,
                    Eigen::Vector3d& com) {
    if (point_cloud.empty()) {
        std::cerr << "Error: Point cloud is empty." << std::endl;
        com = Eigen::Vector3d::Zero(); // Return zero vector if the point cloud is empty
        return;
    }

    Eigen::Vector3d accum_pts = Eigen::Vector3d::Zero();

    for(auto& vec: point_cloud) {
        accum_pts += vec;
    }

    com = accum_pts / static_cast<double>(point_cloud.size()); 
}
