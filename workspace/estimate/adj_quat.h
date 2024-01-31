#ifndef ESTIMATE_ADJ_QUAT_H_
#define ESTIMATE_ADJ_QUAT_H_

#include <Eigen/Dense>
#include <vector>

/**
 * @brief first step of DRaM method for 3D-2D, construct cross-covariance matrix
 * and calculate their determinants
*/
void MakeDeterminants2D(const std::vector<Eigen::Vector3d>& points3D, 
                        const std::vector<Eigen::Vector2d>& points2D, 
                        std::vector<double>& dets);

/**
 * @brief the R_tilde matrix from the DRaM
*/
void MakeRTilde(const std::vector<Eigen::Vector3d>& points3D, 
                const std::vector<Eigen::Vector2d>& points2D,
                Eigen::Matrix3d& r_tilde); 

/**
 * @brief construting M matrix for BI correction
*/
void MMatrix(const std::vector<Eigen::Vector3d>& points3D, 
             const std::vector<Eigen::Vector2d>& points2D,
             Eigen::Matrix4d& M);
