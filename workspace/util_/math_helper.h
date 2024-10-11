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
 * @brief The error is measured in terms of angles (in degrees), calculated from the 
 * dot product of corresponding axes of two rotation matrices.
 */
double MaxAxisAlign(const Eigen::Matrix3d& rot_gt, const Eigen::Matrix3d& rot_est);

/**
 * @brief The dot product of the two quaternions, which measures 
 * the cosine of the half-angle between the two rotations.
 */ 
double CosineDifference(const Eigen::Vector4d& quat_gt, const Eigen::Vector4d& quat_est);


/**
 * @brief calculate relative translation difference 
 * from the g.t. and estimated extrinsic matrix
*/
double RelativeTransErr(const Eigen::Vector3d& gt, const Eigen::Vector3d& estimate);

/**
 * @brief calculate the CoM of the presented cloud
 */
void CalculateCoM(const std::vector<Eigen::Vector3d>& point_cloud,
                  Eigen::Vector3d& com);
