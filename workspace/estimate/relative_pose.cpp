#include <ceres/ceres.h>

#include "base/essential_matrix.h"
#include "base/pose.h"

#include "estimators/essential_matrix.h"
#include "optim/ransac.h"

#include "relative_pose.h"


size_t RelativePoseWMask(const colmap::RANSACOptions& ransac_options,
                        const std::vector<Eigen::Vector2d>& points1,
                        const std::vector<Eigen::Vector2d>& points2,
                        Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                        std::vector<char>* inlier_mask) {
    colmap::RANSAC<colmap::EssentialMatrixFivePointEstimator> ransac(ransac_options);
    const auto report = ransac.Estimate(points1, points2);

    if (!report.success) {
        return 0;
    }

    std::vector<Eigen::Vector2d> inliers1(report.support.num_inliers);
    std::vector<Eigen::Vector2d> inliers2(report.support.num_inliers);

    size_t j = 0;
    for (size_t i = 0; i < points1.size(); ++i) {
        if (report.inlier_mask[i]) {
        inliers1[j] = points1[i];
        inliers2[j] = points2[i];
        j += 1;
        }
    }

    Eigen::Matrix3d R;

    std::vector<Eigen::Vector3d> points3D;
    colmap::PoseFromEssentialMatrix(report.model, inliers1, inliers2, &R, tvec,
                            &points3D);

    *qvec = colmap::RotationMatrixToQuaternion(R);

    if (qvec->hasNaN() || tvec->hasNaN()) {
        return 0;
    }

    inlier_mask->clear();

    *inlier_mask = report.inlier_mask;
    
    return points3D.size();
}
