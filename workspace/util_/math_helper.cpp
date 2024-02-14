#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "util/types.h"

#include <random>

double frobeniusNormRot(const Eigen::Matrix3d& estimated, const Eigen::Matrix3d& gt) {
    return (estimated - gt).norm();
}

double frobeniusNormExt(const Eigen::Matrix3x4d& estimated, const Eigen::Matrix3x4d& gt) {
    return (estimated - gt).norm();
}

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
