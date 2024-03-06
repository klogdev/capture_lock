#include <Eigen/Core>

#include "base/image.h"
#include "base/camera.h"
#include "base/pose.h"

#include "estimators/absolute_pose.h"
#include "optim/ransac.h"

bool PnPEstimation(const colmap::RANSACOptions& options,
                   const std::vector<Eigen::Vector2d>& points2D,
                   const std::vector<Eigen::Vector3d>& points3D,
                   Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                   colmap::Camera* camera, size_t* num_inliers,
                   std::vector<char>* inlier_mask) {
    // preprocess the points2D by normalizing it into the camera space
    std::vector<Eigen::Vector2d> normalize2D;

    for(const auto& p : points2D) {
        Eigen::Vector2d normalized = camera->ImageToWorld(p);
        normalize2D.push_back(normalized);
    }

    colmap::RANSAC<colmap::EPNPEstimator> ransac(options);
    const auto report = ransac.Estimate(normalize2D, points3D);

    Eigen::Matrix3x4d proj_matrix = report.model;
    *num_inliers = report.support.num_inliers;
    *inlier_mask = report.inlier_mask;
    
    // Extract pose parameters.
    *qvec = colmap::RotationMatrixToQuaternion(proj_matrix.leftCols<3>());
    *tvec = proj_matrix.rightCols<1>();

    return true;
}