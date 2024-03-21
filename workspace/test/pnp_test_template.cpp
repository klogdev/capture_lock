
#include <Eigen/Core>
#include <iostream>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

#include "test/pnp_test_template.h"

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
        case EstimatorType::LHM: {
            LHMEstimator::setGlobalOptions(options_.lhm_opt); // Set global LHM options
            std::cout << "current ransac option inside estimator is: " << options_.use_ransac << std::endl;

            LHMEstimator estimator;
            estimated_extrinsic = estimator.Estimate(points2D, points3D);
            if (residuals) {
                estimator.Residuals(points2D, points3D, estimated_extrinsic[0], residuals);
            }
            break;
        }
        case EstimatorType::DRaM_GN: {
            options_.lhm_opt.rot_init_est = "dram";
            options_.lhm_opt.optim_option = "gn";
            LHMEstimator::setGlobalOptions(options_.lhm_opt); // Set global LHM options

            LHMEstimator estimator;
            estimated_extrinsic = estimator.Estimate(points2D, points3D);
            if (residuals) {
                estimator.Residuals(points2D, points3D, estimated_extrinsic[0], residuals);
            }
            break;
        }
        case EstimatorType::DRaM_LHM: {
            options_.lhm_opt.rot_init_est = "dram";
            options_.lhm_opt.optim_option = "lhm";
            LHMEstimator::setGlobalOptions(options_.lhm_opt); // Set global LHM options

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
        case EstimatorType::LHM: {
            colmap::RANSACOptions options;
            options.max_error = 1e-5;
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
            options_.lhm_opt.rot_init_est = "dram";
            options_.lhm_opt.optim_option = "lhm";
            LHMEstimator::setGlobalOptions(options_.lhm_opt); // Set global LHM options
            
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
         case EstimatorType::DRaM_GN: {
            options_.lhm_opt.rot_init_est = "dram";
            options_.lhm_opt.optim_option = "gn";
            LHMEstimator::setGlobalOptions(options_.lhm_opt); // Set global LHM options
            
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


