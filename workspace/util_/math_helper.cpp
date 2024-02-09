#include <Eigen/Core>
#include "util/types.h"

double frobeniusNorm(const Eigen::Matrix3x4d& estimated, const Eigen::Matrix3x4d& gt) {
    return (estimated - gt).norm();
}
