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
                         std::vector<char>* inlier_mask,
                         std::vector<Eigen::Vector3d>* points3D) {
    // first convert the feature/pixel points to homogeneaous points
    std::vector<Eigen::Vector2d> homo1;
    std::vector<Eigen::Vector2d> homo2;

    for(int i = 0; i < points1.size(); i++) {
        homo1.push_back(camera.ImageToWorld(points1[i]));
        homo2.push_back(camera.ImageToWorld(points2[i]));
    }

    std::cout << "check image to world points" << std::endl;
    for(int i = 0; i < 10; i++) {
        std::cout << i << "th homo pair is" << std::endl;
        std::cout << homo1[i] << std::endl;
        std::cout << homo2[i] << std::endl; 
    }

    colmap::RANSAC<colmap::EssentialMatrixFivePointEstimator> ransac(ransac_options);
    const auto report = ransac.Estimate(homo1, homo2);

    // check residuals of the model
    std::vector<double> residuals;
    colmap::EssentialMatrixFivePointEstimator::Residuals(homo1, homo2, report.model,
                                               &residuals);
    std::cout << "check 5 points estimated averaged residuals" << std::endl;
    
    double avg_residual = std::accumulate(residuals.begin(), residuals.end(), 0.0) / residuals.size();
    std::cout << avg_residual << std::endl;

    if (!report.success) {
        return 0;
    }

    std::vector<Eigen::Vector2d> inliers1(report.support.num_inliers);
    std::vector<Eigen::Vector2d> inliers2(report.support.num_inliers);

    size_t j = 0;
    for (size_t i = 0; i < points1.size(); i++) {
        if (report.inlier_mask[i]) {
            inliers1[j] = homo1[i];
            inliers2[j] = homo2[i];
            j += 1;
        }
    }
    std::cout << "total num of inliers from 5 points is: " << report.support.num_inliers << std::endl;
    std::cout << "total num of estimation is: " << homo1.size() << std::endl;

    Eigen::Matrix3d R;

    colmap::PoseFromEssentialMatrix(report.model, inliers1, inliers2, &R, tvec,
                            points3D);
    std::cout << "check the triangulated points from 5-points" << std::endl;
    const std::vector<Eigen::Vector3d>& vec = *points3D;  // Dereference the pointer to get the vector

    for(int i = 0; i < 10; i++) {
        std::cout << i << "th triangulated point" << std::endl;
        std::cout << vec[i] << std::endl;
    }

    *qvec = colmap::RotationMatrixToQuaternion(R);

    if (qvec->hasNaN() || tvec->hasNaN()) {
        return 0;
    }

    inlier_mask->clear();

    *inlier_mask = report.inlier_mask;
    
    // here the num of inliers are ones passed the Cheirality test
    return points3D->size();
}
