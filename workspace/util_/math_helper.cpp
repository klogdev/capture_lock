#include <Eigen/Core>
#include "util/types.h"

double frobeniusNormRot(const Eigen::Matrix3d& estimated, const Eigen::Matrix3d& gt) {
    return (estimated - gt).norm();
}

double frobeniusNormExt(const Eigen::Matrix3x4d& estimated, const Eigen::Matrix3x4d& gt) {
    return (estimated - gt).norm();
}

