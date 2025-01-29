#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

#include "estimators/utils.h"
#include "estimate/sqpnp.h"
#include "estimate/seq_quad.h"
#include "estimate/sq_types.h"

std::vector<SQPnPEstimator::M_t> SQPnPEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
    std::vector<M_t> models;

    if(points2D.size() < kMinNumSamples || points3D.size() < kMinNumSamples) {
        return models;
    }

    SQPnPEstimator sqpnp;
    M_t proj_matrix;

    if (!sqpnp.ComputeSQPnPPose(points2D, points3D, &proj_matrix)) {
        return std::vector<SQPnPEstimator::M_t>({});
    }

    return std::vector<SQPnPEstimator::M_t>({proj_matrix});
}

bool SQPnPEstimator::ComputeSQPnPPose(const std::vector<Eigen::Vector2d>& points2D,
                           const std::vector<Eigen::Vector3d>& points3D,
                           Eigen::Matrix3x4d* proj_matrix) {
    
    if (points2D.size() != points3D.size() || points2D.size() < 3) {
        return false;
    }

    // Convert to solver's internal types
    std::vector<_Point> points;
    std::vector<_Projection> projections;
    
    points.reserve(points3D.size());
    projections.reserve(points2D.size());
    
    for (size_t i = 0; i < points3D.size(); i++) {
        // Direct assignment using Eigen - no conversion needed
        points.emplace_back()._Point::operator=(points3D[i]);
        projections.emplace_back()._Projection::operator=(points2D[i]);
    }

    // Create and run solver
    SQPnPSolver solver(points, projections);
    
    if (solver.IsValid()) {
        solver.Solve();
        
        if (solver.NumberOfSolutions() > 0) {
            const SQPSolution* solution = solver.SolutionPtr(0);
            
            // Convert to 3x4 projection matrix
            // First 3x3 is rotation
            proj_matrix->block<3,3>(0,0) = Eigen::Map<const Eigen::Matrix3d>(solution->r_hat.data());
            // Last column is translation
            proj_matrix->block<3,1>(0,3) = solution->t;
            
            return true;
        }
    }
    return false;
}

void SQPnPEstimator::Residuals(const std::vector<X_t>& points2D,
                              const std::vector<Y_t>& points3D,
                              const M_t& proj_matrix, std::vector<double>* residuals) {
    residuals->clear();
    colmap::ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}