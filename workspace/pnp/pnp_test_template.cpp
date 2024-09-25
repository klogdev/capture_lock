
#include <Eigen/Core>
#include <iostream>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"
#include "estimate/dls.h"

#include "pnp/pnp_test_template.h"

// Function to perform estimation
bool EstimatorWrapper::estimate(const std::vector<Eigen::Vector2d>& points2D,
                const std::vector<Eigen::Vector3d>& points3D,
                std::vector<Eigen::Matrix3x4d>& estimated_extrinsic,
                std::vector<double>* residuals) {
    if (options_.use_ransac) {
        return runWithRansac(points2D, points3D, estimated_extrinsic, residuals);
    } else {
        return runStandalone(points2D, points3D, estimated_extrinsic, residuals);
    }
}

// Runs the lhm-estimator standalone
bool EstimatorWrapper::runStandalone(const std::vector<Eigen::Vector2d>& points2D,
                                     const std::vector<Eigen::Vector3d>& points3D,
                                     std::vector<Eigen::Matrix3x4d>& estimated_extrinsic,
                                     std::vector<double>* residuals) {

    switch (type_) {
        case EstimatorType::EPnP: {
            colmap::EPNPEstimator estimator;
            estimated_extrinsic = estimator.Estimate(points2D, points3D);
            if (residuals) {
                estimator.Residuals(points2D, points3D, estimated_extrinsic[0], residuals);
            }
            break;
        }
        case EstimatorType::DLS: {
            DLSEstimator estimator;
            estimated_extrinsic = estimator.Estimate(points2D, points3D);
            if(residuals) {
                estimator.Residuals(points2D, points3D, estimated_extrinsic[0], residuals);
            }
            break;
        }
        case EstimatorType::LHM: {
            LHMEstimator::options_.rot_init_est = "horn"; // Set global LHM options
            std::cout << "current ransac option inside estimator is: " << options_.use_ransac << std::endl;

            LHMEstimator estimator;
            estimated_extrinsic = estimator.Estimate(points2D, points3D);
            if (residuals) {
                estimator.Residuals(points2D, points3D, estimated_extrinsic[0], residuals);
            }
            break;
        }
        case EstimatorType::DRaM_GN: {
            LHMEstimator::options_.rot_init_est = "dram";
            LHMEstimator::options_.optim_option = "gn";

            LHMEstimator estimator;
            estimated_extrinsic = estimator.Estimate(points2D, points3D);
            if (residuals) {
                estimator.Residuals(points2D, points3D, estimated_extrinsic[0], residuals);
            }
            break;
        }
        case EstimatorType::DRaM_LHM: {
            LHMEstimator::options_.rot_init_est = "dram";
            LHMEstimator::options_.optim_option = "lhm";

            LHMEstimator estimator;
            estimated_extrinsic = estimator.Estimate(points2D, points3D);
            if (residuals) {
                estimator.Residuals(points2D, points3D, estimated_extrinsic[0], residuals);
            }
            break;
        }
        // Handle unsupported types
        default:
            std::cerr << "unsupported type of PnP estimator!" << std::endl;
            return false; 
    }
    return true;
}

// Runs the lhm-estimator within RANSAC
bool EstimatorWrapper::runWithRansac(const std::vector<Eigen::Vector2d>& points2D,
                                     const std::vector<Eigen::Vector3d>& points3D,
                                     std::vector<Eigen::Matrix3x4d>& estimated_extrinsic,
                                     std::vector<double>* residuals) {
    switch (type_)
    {
        case EstimatorType::EPnP: {
            colmap::RANSACOptions options;
            options.max_error = 1e-5;
            options.min_inlier_ratio = 0.02;
            options.max_num_trials = 10000;
            colmap::RANSAC<colmap::EPNPEstimator> ransac(options);
            const auto report = ransac.Estimate(points2D, points3D);

            std::cout << "current w/ ransac & epnp estimate" << std::endl;

            if(report.success == true) {
                std::cout << "current ransac passed" << std::endl; 
            }
            else {
                std::cout << "current ransac failed" << std::endl; 
            }

            estimated_extrinsic = {report.model};
            if (residuals) {
                colmap::EPNPEstimator::Residuals(points2D, points3D, report.model, residuals);
            }
            break;
        }
        case EstimatorType::DLS: {
            colmap::RANSACOptions options;
            options.max_error = 1e-5;
            options.min_inlier_ratio = 0.02;
            options.max_num_trials = 10000;
            colmap::RANSAC<DLSEstimator> ransac(options);
            const auto report = ransac.Estimate(points2D, points3D);

            std::cout << "current w/ ransac & dls estimate" << std::endl;
            std::cout << "current ransac option inside estimator is: " << options_.use_ransac << std::endl;


            if(report.success == true) {
                std::cout << "current ransac passed" << std::endl; 
            }
            else {
                std::cout << "current ransac failed" << std::endl; 
            }

            estimated_extrinsic = {report.model};
            if (residuals) {
                DLSEstimator::Residuals(points2D, points3D, report.model, residuals);
            }
            break;
        }
        case EstimatorType::LHM: {
            colmap::RANSACOptions options;
            options.max_error = 1e-5;
            options.min_inlier_ratio = 0.02;
            options.max_num_trials = 10000;
            colmap::RANSAC<LHMEstimator> ransac(options);
            const auto report = ransac.Estimate(points2D, points3D);

            std::cout << "current w/ ransac & lhm estimate" << std::endl;
            std::cout << "current ransac option inside estimator is: " << options_.use_ransac << std::endl;


            if(report.success == true) {
                std::cout << "current ransac passed" << std::endl; 
            }
            else {
                std::cout << "current ransac failed" << std::endl; 
            }

            estimated_extrinsic = {report.model};
            if (residuals) {
                LHMEstimator::Residuals(points2D, points3D, report.model, residuals);
            }
            break;
        }
        case EstimatorType::DRaM_LHM: {
            LHMEstimator::options_.rot_init_est = "dram";
            LHMEstimator::options_.optim_option = "lhm";
            
            colmap::RANSACOptions options;
            options.max_error = 1e-5;
            options.min_inlier_ratio = 0.02;
            options.max_num_trials = 10000;
            colmap::RANSAC<LHMEstimator> ransac(options);
            const auto report = ransac.Estimate(points2D, points3D);

            if(report.success == true) {
                std::cout << "current ransac passed" << std::endl; 
            }
            else {
                std::cout << "current ransac failed" << std::endl; 
            }

            estimated_extrinsic = {report.model};
            if (residuals) {
                LHMEstimator::Residuals(points2D, points3D, report.model, residuals);
            }
            break;
        }
         case EstimatorType::DRaM_GN: {
            LHMEstimator::options_.rot_init_est = "dram";
            LHMEstimator::options_.optim_option = "gn";
            
            colmap::RANSACOptions options;
            options.max_error = 1e-5;
            colmap::RANSAC<LHMEstimator> ransac(options);
            const auto report = ransac.Estimate(points2D, points3D);

            if(report.success == true) {
                std::cout << "current ransac passed" << std::endl; 
            }
            else {
                std::cout << "current ransac failed" << std::endl; 
            }

            estimated_extrinsic = {report.model};
            if (residuals) {
                LHMEstimator::Residuals(points2D, points3D, report.model, residuals);
            }
            break;
        }
        default:
            std::cerr << "unsupported type of PnP estimator!" << std::endl;
            break;
    }
    return true; 
}


