#include <Eigen/Core>
#include "base/triangulation.h"
#include "base/image.h"
#include "base/camera.h"

/**
 * @brief DLT triangulation by invoking colmap's functions
*/
void TriangulateImage(const colmap::Image& frame1, const colmap::Image& frame2,
                      const colmap::Camera& camera,
                      const std::vector<Eigen::Vector2d>& match_vec1,
                      const std::vector<Eigen::Vector2d>& match_vec2,
                      std::vector<Eigen::Vector3d>& triangulated);

/**
 * @brief check whether the triangulated point is valid 
 * by its depth and relative angle between 2 images
*/
bool CheckTriangulateQuality(const Eigen::Matrix3x4d& proj_mat1, 
                             const Eigen::Matrix3x4d& proj_mat2,
                             const Eigen::Vector3d& proj_center1, 
                             const Eigen::Vector3d& proj_center2,
                             Eigen::Vector3d& xyz, double min_ang);