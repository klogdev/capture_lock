#include <Eigen/Core>
#include <random>

#include "util/types.h"


/**
 * @brief estimation norm between two matrices
 * could be 3x3, i.e. rotation matrix or 3x4, i.e. extrinsic matrix
*/
template<typename MatrixType>
double frobeniusNorm(const MatrixType& estimated, const MatrixType& gt) {
    return (estimated - gt).norm();
}

/**
 * @brief generate random rotation from SO3 manifold as an unit quaternion
*/
Eigen::Vector4d GenRandomRot();

/**
 * @brief generate random data from an uniform distribution
 * with bound
*/
double RandomUniform(double x_ini, double x_end);

/**
 * @brief generate random data from a gaussian distribution
 * with mean and std
*/
double RandomGaussian(double mean, double std);

/**
 * @brief calculate relative quaternion difference
 * from the g.t. and estimated extrinsic matrix
*/
double RelativeQuatErr(const Eigen::Vector4d& quat_gt, const Eigen::Vector4d& quat_est);

/**
 * @brief calculate relative translation difference 
 * from the g.t. and estimated extrinsic matrix
*/
double RelativeTransErr(const Eigen::Vector3d& gt, const Eigen::Vector3d& estimate);
