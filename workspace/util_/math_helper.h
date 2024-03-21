#include <Eigen/Core>
#include <random>

#include "util/types.h"

/**
 * @brief estimation norm between two rotational matrices
*/
double frobeniusNormRot(const Eigen::Matrix3d& estimated, const Eigen::Matrix3d& gt);

/**
 * @brief estimation norm between two extrinsic matrices
*/
double frobeniusNormExt(const Eigen::Matrix3x4d& estimated, const Eigen::Matrix3x4d& gt);

/**
 * @brief generate random rotation for SO3 manifold as an unit quaternion
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
double RelativeQuatErr(const Eigen::Matrix3x4d& gt, const Eigen::Matrix3x4d& estimate);

/**
 * @brief calculate relative translation difference 
 * from the g.t. and estimated extrinsic matrix
*/
double RelativeTransErr(const Eigen::Matrix3x4d& gt, const Eigen::Matrix3x4d& estimate);
