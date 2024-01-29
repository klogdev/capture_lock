#include <Eigen/Core>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

// hack test fn from absolute_pose_test.cpp
int main() {
    colmap::SetPRNGSeed(0);

    std::vector<Eigen::Vector3d> points3D;
    points3D.emplace_back(1, 1, 1);
    points3D.emplace_back(0, 1, 1);
    points3D.emplace_back(3, 1.0, 4);
    points3D.emplace_back(3, 1.1, 4);
    points3D.emplace_back(3, 1.2, 4);
    points3D.emplace_back(3, 1.3, 4);
    points3D.emplace_back(3, 1.4, 4);
    points3D.emplace_back(2, 1, 7);

    auto points3D_faulty = points3D;
    for (size_t i = 0; i < points3D.size(); ++i) {
        points3D_faulty[i](0) = 20;
    }

    for (double qx = 0; qx < 0.3; qx += 0.2) {
        for (double tx = 0; tx < 0.2; tx += 0.1) {
            const colmap::SimilarityTransform3 orig_tform(1, Eigen::Vector4d(1, qx, 0, 0),
                                                          Eigen::Vector3d(tx, 0, 0));

            // Project points to camera coordinate system
            // here we project the original 3D points
            std::vector<Eigen::Vector2d> points2D;
            for (size_t i = 0; i < points3D.size(); ++i) {
                Eigen::Vector3d point3D_camera = points3D[i];
                orig_tform.TransformPoint(&point3D_camera);
                points2D.push_back(point3D_camera.hnormalized());
            }

            colmap::RANSACOptions options;
            options.max_error = 1e-5;
            colmap::RANSAC<LHMEstimator> ransac(options);
            const auto report = ransac.Estimate(points2D, points3D);

            if(report.success == true){
                std::cout << "current ransac passed" << std::endl; 
            }
            else {
                std::cout << "current ransac failed" << std::endl; 
            }

            // Test if correct transformation has been determined.
            const double matrix_diff =
                (orig_tform.Matrix().topLeftCorner<3, 4>() - report.model).norm();
            std::cout << "current estimated matrix: " << std::endl;
            std::cout << report.model << std::endl;
            std::cout << "current matrix norm: " << std::endl;
            std::cout << matrix_diff << std::endl;
            if(matrix_diff < 1e-2){
                std::cout << "current Frobenius norm as expected" << std::endl;
            }
            else{
                std::cout << "current Frobenius norm is abnormal" << std::endl;
            }

            // Test residuals of exact points.
            std::vector<double> residuals;
            LHMEstimator::Residuals(points2D, points3D, report.model, &residuals);
            for (size_t i = 0; i < residuals.size(); ++i) {
                std::cout <<"residual of the original point: " << std::endl;
                std::cout << residuals[i] << std::endl;
                if(residuals[i] < 1e-3){
                    std::cout << "residuals of exact point " << i << " is reasonable" << std::endl;
                }
            }   

            // Test residuals of faulty points.
            LHMEstimator::Residuals(points2D, points3D_faulty, report.model,
                                    &residuals);
            for (size_t i = 0; i < residuals.size(); ++i) {
                if(residuals[i] > 0.1){
                    std::cout << "residuals of faulty point " << i << " is reasonable" << std::endl;
                    std::cout << residuals[i] << std::endl;
                }
            }
        }
    }

    return 0;
}



