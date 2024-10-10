#ifndef PNP_PNP_HELPER_H_
#define PNP_PNP_HELPER_H_

#include <Eigen/Core>
#include <unordered_map>
#include <string>
#include <memory>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

/**
 * @brief preprocessing of the EPnP box corner data generation
 * @arg  3D points in camera space
*/
void EPnPBoxCorner(std::vector<Eigen::Vector3d>& camera_space_points);

/**
 * @brief 
 * preprocessing of a planar data generation comparable to EPnP box corners
 * @arg 3D points in camera space
 */
void EPnPPlanar(std::vector<Eigen::Vector3d>& camara_space_points);

/**
 * @brief pass the intrinsic matrix by reference
 */
void GetIntrinsic(Eigen::Matrix3d& k);

/**
 * @brief preprocessing of the EPnP random data generation
 * inside the box [-2,2]x[-2,2]x[4,8]
*/
void EPnPInsideRand(std::vector<Eigen::Vector3d>& camera_space_points,
                    int num_pts);

/**
 * @brief generate a random rotation matrix and pass by reference
*/
void EPnPRandomRot(Eigen::Matrix3d& rot);

/**
 * @brief generate a random translation vector and pass by reference
 */
void EPnPRandomTrans(Eigen::Vector3d& trans);

/**
 * @brief translate points inside the camera space
 */
void CameraSpaceShift(const std::vector<Eigen::Vector3d>& camera_pts, 
                      const Eigen::Vector3d& trans,
                      std::vector<Eigen::Vector3d>& shifted_pts);

/**
 * @brief set the intrinsic matrix to convert the pixel to camera space
 * if use film plane data, set the intrinsic as an identity by default
*/
void SetIntrinsic(std::string calib_path, Eigen::Matrix3d& calib_mat);

/**
 * @brief generate noised 2d set from a list of camera space points
 */
void GenOneSetNoise2D(std::vector<Eigen::Vector3d>& camera_space_points, 
                      std::vector<Eigen::Vector2d>& one_set_2d,
                      Eigen::Matrix3d& k, double sigma);

/**
 * @brief add small perturbation to each camera space point
 */
void Perturbation3D(std::vector<Eigen::Vector3d>& camera_space_points,
                    double sigma);

/**
 * @brief replace part of 2D points as outliers
 * we follow the EPnP data where the aspect ratio for image is 640, 480
 */
void AddOutlier2D(std::vector<Eigen::Vector2d>& points2D, double outlier_rate, 
                  const int image_x, const int image_y, Eigen::Matrix3d& k_inv);

#endif // PNP_PNP_HELPER_H_
