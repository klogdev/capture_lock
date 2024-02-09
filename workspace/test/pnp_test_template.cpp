
#include <Eigen/Core>
#include <iostream>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

#include "test/pnp_test_template.h"

// Function to perform estimation
bool LHMEstimatorWrapper::estimate(const std::vector<X_t>& points2D,
                const std::vector<Y_t>& points3D,
                M_t& estimated_extrinsic,
                std::vector<double>* residuals = nullptr) {
    if (options_.use_ransac) {
        return runWithRansac(points2D, points3D, estimated_extrinsic, residuals);
    } else {
        return runStandalone(points2D, points3D, estimated_extrinsic, residuals);
    }
}

// Runs the lhm-estimator standalone
bool LHMEstimatorWrapper::runStandalone(const std::vector<X_t>& points2D,
                                     const std::vector<Y_t>& points3D,
                                     M_t& estimated_extrinsic,
                                     std::vector<double>* residuals) {
    Estimator::setGlobalOptions(options_.lhm_opt); // Set global LHM options
    if (options_.gt_pose != nullptr) {
        Estimator::setGroundTruthPose(options_.gt_pose);
    }

    Estimator estimator;
    estimated_extrinsic = estimator.Estimate(points2D, points3D);
    if (residuals) {
        estimator.Residuals(points2D, points3D, estimated_extrinsic, residuals);
    }

    switch (type_) {
        case EstimatorType::EPnP: {
            colmap::EPNPEstimator estimator;
            estimated_extrinsic = estimator.Estimate(points2D, points3D);
            if (residuals) {
                estimator.Residuals(points2D, points3D, estimated_extrinsic, residuals);
            }
            break;
        }
        case EstimatorType::DRaM_GN: {
            options_.lhm_opt.rot_init_est = "dram";
            options_.lhm_opt.optim_option = "gn";
            LHMEstimator::setGlobalOptions(options_.lhm_opt); // Set global LHM options
            if (options_.gt_pose != nullptr) {
                LHMEstimator::setGroundTruthPose(options_.gt_pose);
            }

            LHMEstimator estimator;
            estimated_extrinsic = estimator.Estimate(points2D, points3D);
            if (residuals) {
                estimator.Residuals(points2D, points3D, estimated_extrinsic, residuals);
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

// Runs the estimator within RANSAC
bool LHMEstimatorWrapper::runWithRansac(const std::vector<X_t>& points2D,
                                     const std::vector<Y_t>& points3D,
                                     M_t& estimated_extrinsic,
                                     std::vector<double>* residuals) {
    Estimator::setGlobalOptions(options_.lhm_opt); // Set global LHM options
    if (options_.gt_pose != nullptr) {
        Estimator::setGroundTruthPose(options_.gt_pose);
    }

    colmap::RANSACOptions options;
    options.max_error = 1e-5;
    colmap::RANSAC<Estimator> ransac(options);
    const auto report = ransac.Estimate(points2D, points3D);

    if(report.success == true) {
        std::cout << "current ransac passed" << std::endl; 
    }
    else {
        std::cout << "current ransac failed" << std::endl; 
    }

    estimated_extrinsic = report.model;
    if (residuals) {
        Estimator::Residuals(points2D, points3D, report.model, residuals);
    }
    switch (type_)
    {
        case EstimatorType::EPnP: {
            colmap::RANSACOptions options;
            options.max_error = 1e-5;
            colmap::RANSAC<colmap::EPNPEstimator> ransac(options);
            const auto report = ransac.Estimate(points2D, points3D);

            if(report.success == true) {
                std::cout << "current ransac passed" << std::endl; 
            }
            else {
                std::cout << "current ransac failed" << std::endl; 
            }

            estimated_extrinsic = report.model;
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

            if(report.success == true) {
                std::cout << "current ransac passed" << std::endl; 
            }
            else {
                std::cout << "current ransac failed" << std::endl; 
            }

            estimated_extrinsic = report.model;
            if (residuals) {
                LHMEstimator::Residuals(points2D, points3D, report.model, residuals);
            }
            break;
        }
        default:
            std::cerr << "unsupported type of PnP estimator!" << std::endl;
            break;
    }
    return false; 
}

// Function to perform estimation
bool EstimatorWrapper::estimate(const std::vector<X_t>& points2D,
                const std::vector<Y_t>& points3D,
                M_t& estimated_extrinsic,
                std::vector<double>* residuals = nullptr) {
    if (options_.use_ransac) {
        return runWithRansac(points2D, points3D, estimated_extrinsic, residuals);
    } else {
        return runStandalone(points2D, points3D, estimated_extrinsic, residuals);
    }
}

// Runs the estimator standalone
bool EstimatorWrapper::runStandalone(const std::vector<X_t>& points2D,
                                     const std::vector<Y_t>& points3D,
                                     M_t& estimated_extrinsic,
                                     std::vector<double>* residuals) {
    
    Estimator estimator;
    estimated_extrinsic = estimator.Estimate(points2D, points3D);
    if (residuals) {
        estimator.Residuals(points2D, points3D, estimated_extrinsic, residuals);
    }
    return true;
}