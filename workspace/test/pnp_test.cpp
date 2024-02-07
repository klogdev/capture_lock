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
    std::vector<Eigen::Vector2d> points2D;
    std::vector<Eigen::Matrix3x4d> poses;


    for (size_t i = 0; i < points2D.size(); i++) {

        Eigen::Matrix3x4d manual_proj; // debug without ransac
        LHMEstimator lhm;
        bool curr_est = lhm.ComputeLHMPose(points2D[i], points3D[i], &manual_proj);
        std::cout << "manually estimated lhm" << std::endl;
        std::cout << manual_proj << std::endl;

        colmap::RANSACOptions options;
        options.max_error = 1e-5;
        colmap::RANSAC<LHMEstimator> ransac(options);
        // colmap::RANSAC<colmap::EPNPEstimator> ransac(options);
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
        // colmap::EPNPEstimator::Residuals(points2D, points3D, report.model, &residuals);
        for (size_t i = 0; i < residuals.size(); i++) {
            std::cout <<"residual of the original point: " << std::endl;
            std::cout << residuals[i] << std::endl;
            if(residuals[i] < 1e-3){
                std::cout << "residuals of exact point " << i << " is reasonable" << std::endl;
            }
        }   
        
    }

    return 0;
}



