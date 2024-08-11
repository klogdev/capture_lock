#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <iostream>
#include <boost/format.hpp>

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

    std::cout << "relative quat error inside fn is: " << boost::format("%.15f") %(diff/est_norm) << std::endl;
    return diff/est_norm;
}

double RelativeTransErr(const Eigen::Vector3d& gt, const Eigen::Vector3d& estimate) {
    double diff = (gt - estimate).norm();
    double est_norm = estimate.norm();

    return diff/est_norm;
}
