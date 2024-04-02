#include <ceres/ceres.h>

#include "base/essential_matrix.h"
#include "base/pose.h"
#include "base/camera.h"

#include "estimators/essential_matrix.h"
#include "optim/ransac.h"

#include "estimate/relative_pose.h"


size_t RelativePoseWMask(const colmap::RANSACOptions& ransac_options,
                         colmap::Camera& camera,
                         const std::vector<Eigen::Vector2d>& points1,
                         const std::vector<Eigen::Vector2d>& points2,
                         Eigen::Vector4d* qvec, Eigen::Vector3d* tvec,
                         std::vector<char>* inlier_mask) {
    // first convert the feature/pixel points to homogeneaous points
    std::vector<Eigen::Vector2d> homo1;
    std::vector<Eigen::Vector2d> homo2;

    for(int i = 0; i < points1.size(); i++) {
        homo1.push_back(camera.ImageToWorld(points1[i]));
        homo2.push_back(camera.ImageToWorld(points2[i]));
    }
    colmap::RANSAC<colmap::EssentialMatrixFivePointEstimator> ransac(ransac_options);
    const auto report = ransac.Estimate(homo1, homo2);

    // check residuals of the model
    std::vector<double> residuals;
    colmap::EssentialMatrixFivePointEstimator::Residuals(homo1, homo2, report.model,
                                               &residuals);
    std::cout << "check 5 points estimated residuals" << std::endl;
    for (size_t i = 0; i < residuals.size(); ++i) {
        std::cout << residuals[i] << std::endl;
    }

    if (!report.success) {
        return 0;
    }

    std::vector<Eigen::Vector2d> inliers1(report.support.num_inliers);
    std::vector<Eigen::Vector2d> inliers2(report.support.num_inliers);

    size_t j = 0;
    for (size_t i = 0; i < points1.size(); ++i) {
        if (report.inlier_mask[i]) {
            inliers1[j] = homo1[i];
            inliers2[j] = homo2[i];
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
    
    // here the num of inliers are ones passed the Cheirality test
    return points3D.size();
}
