#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

#include "estimators/utils.h"

#include "estimate/reppnp.h"
#include "estimate/reppnp_raw.h"
#include <cmath>
#include <algorithm>
#include <numeric>


std::vector<REPPnPEstimator::M_t> REPPnPEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
    std::vector<M_t> models;

    if(points2D.size() < kMinNumSamples || points3D.size() < kMinNumSamples) {
        return models;
    }

    REPPnPEstimator reppnp;
    M_t proj_matrix;

    if (!reppnp.ComputeREPPnPPose(points2D, points3D, &proj_matrix)) {
        return std::vector<REPPnPEstimator::M_t>({});
    }

    return std::vector<REPPnPEstimator::M_t>({proj_matrix});
}

void REPPnPEstimator::Residuals(const std::vector<X_t>& points2D,
                                const std::vector<Y_t>& points3D,
                                const M_t& proj_matrix, std::vector<double>* residuals) {
    residuals->clear();
    colmap::ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}

bool REPPnPEstimator::ComputeREPPnPPose(
    const std::vector<Eigen::Vector2d>& points2D,
    const std::vector<Eigen::Vector3d>& points3D,
    Eigen::Matrix3x4d* proj_matrix) {
    
    // Input validation
    if (points2D.size() != points3D.size() || points2D.empty() || !proj_matrix) {
        return false;
    }

    // Preallocate memory for REPPnP matrices
    const size_t n_points = points2D.size();
    REPPnP::Coordinates2D_t i_P(2, n_points);
    REPPnP::Coordinates3D_t w_P(3, n_points);
    
    for (size_t i = 0; i < points2D.size(); ++i) {
        i_P.col(i) = points2D[i];
        w_P.col(i) = points3D[i];
    }

    // Setup camera matrix (identity since we assume normalized coordinates)
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();

    // Setup REPPnP options with default values
    REPPnP::REPPnP_Options opts;
    // You can customize options here if needed:
    // opts.dim_kernel = 4;
    // opts.refine = true;
    opts.max_algebraic_error = 0.01;

    try {
        // Run REPPnP
        std::tuple<TF, std::vector<REPPnP::idx_t>> result = 
            REPPnP::REPPnP(i_P, w_P, K, opts);

        // Get results
        const TF& pose = std::get<0>(result);

        // Convert to projection matrix format
        Eigen::Matrix3d R = pose.GetRotmatrix();
        Eigen::Vector3d t = pose.GetTranslation();
        
        // Fill output projection matrix [R|t]
        proj_matrix->block<3,3>(0,0) = R;
        proj_matrix->block<3,1>(0,3) = t;

        return true;
    }
    catch (const std::bad_alloc& e) {
        std::cerr << "Memory allocation failed in REPPnP: " << e.what() << std::endl;
        return false;
    }
    catch (const REPPnP::REPPnPException& e) {
        // Handle any REPPnP-specific exceptions
        return false;
    }
    catch (const std::exception& e) {
        // Handle any other exceptions
        return false;
    }
}