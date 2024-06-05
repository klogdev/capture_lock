#ifndef GEOMETRY_ESSENTIAL_MAT_H_
#define GEOMETRY_ESSENTIAL_MAT_H_

#include <Eigen/Core>

// Perform cheirality constraint test, i.e., determine which of the triangulated
// correspondences lie in front of of both cameras. The first camera has the
// projection matrix P1 = [I | 0] and the second camera has the projection
// matrix P2 = [R | t].
//
// @param R            3x3 rotation matrix of second projection matrix.
// @param t            3x1 translation vector of second projection matrix.
// @param points1      First set of corresponding points.
// @param points2      Second set of corresponding points.
// @param points3D     Points that lie in front of both cameras.
bool CheiralityCheck(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                     const std::vector<Eigen::Vector2d>& points1,
                     const std::vector<Eigen::Vector2d>& points2,
                     std::vector<Eigen::Vector3d>* points3D);

// Recover the most probable pose from the given essential matrix.
//
// The pose of the first image is assumed to be P = [I | 0].
//
// @param E            3x3 essential matrix.
// @param points1      First set of corresponding points.
// @param points2      Second set of corresponding points.
// @param inlier_mask  Only points with `true` in the inlier mask are
//                     considered in the cheirality test. Size of the
//                     inlier mask must match the number of points N.
// @param R            Most probable 3x3 rotation matrix.
// @param t            Most probable 3x1 translation vector.
// @param points3D     Triangulated 3D points infront of camera.
void PoseFromEssential(const Eigen::Matrix3d& E,
                       const std::vector<Eigen::Vector2d>& points1,
                       const std::vector<Eigen::Vector2d>& points2,
                       Eigen::Matrix3d* R, Eigen::Vector3d* t,
                       std::vector<Eigen::Vector3d>* points3D);

#endif // GEOMETRY_ESSENTIAL_MAT_H_
