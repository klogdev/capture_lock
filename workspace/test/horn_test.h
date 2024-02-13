#ifndef TEST_HORN_H_
#define TEST_HORN_H_

#include <vector>

#include <Eigen/Core>

/**
 * @brief calculate relative rotation and translation up to a scale of
 * depth for the initial guess of the pose via Weak Perspective model
 * the registration using closed form method w/ quat and profile matrix:
 * Horn, "Closed-form Solution of Absolute Orientation Using Unit Quaternions",
 * JOSAA (4):4, 1987, pp.629
 * we always use eqn. 39 for the scale w/o any guess
*/
bool HornRot(std::vector<Eigen::Vector3d>& points3D0,
            std::vector<Eigen::Vector3d>& points3D1,
                            Eigen::Matrix3d& R);

/**
 * @brief preprocess of the weak perspective model by getting
 * the centroid of both point clouds.
*/
void GetCentroidUtil(std::vector<Eigen::Vector3d>& points3D0,
                std::vector<Eigen::Vector3d>& points3D1,
                    Eigen::Vector3d& pc, Eigen::Vector3d& qc);

#endif // TEST_HORN_H_
