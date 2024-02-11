#include <Eigen/Core>
#include "util/types.h"

/**
 * @brief estimation norm between two rotational matrices
*/
double frobeniusNormRot(const Eigen::Matrix3d& estimated, const Eigen::Matrix3d& gt);

/**
 * @brief estimation norm between two extrinsic matrices
*/
double frobeniusNormExt(const Eigen::Matrix3x4d& estimated, const Eigen::Matrix3x4d& gt);

