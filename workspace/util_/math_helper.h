#include <Eigen/Core>
#include "util/types.h"

/**
 * @brief estimation norm between matrices
*/
double frobeniusNorm(const Eigen::Matrix3x4d& estimated, const Eigen::Matrix3x4d& gt);
