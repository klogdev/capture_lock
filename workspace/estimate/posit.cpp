#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

#include "estimators/utils.h"
#include "estimate/posit.h"

std::vector<POSITEstimator::M_t> POSITEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D
) {
    POSITEstimator posit;
    M_t proj_matrix;

    if (!posit.ComputePOSITPose(points2D, points3D, &proj_matrix)) {
        return std::vector<POSITEstimator::M_t>({});
    }

    return std::vector<POSITEstimator::M_t>({proj_matrix});
}


bool POSITEstimator::ComputePOSITPose(const std::vector<Eigen::Vector2d>& points2D,
                                      const std::vector<Eigen::Vector3d>& points3D,
                                      Eigen::Matrix3x4d* proj_matrix) {
    if (points2D.size() != points3D.size() || points2D.size() < 4) {
        return false; // POSIT requires at least 4 points
    }

    int num_points = points2D.size();
    
    // Create object matrix (3xN)
    Eigen::MatrixXd object_matrix = Eigen::MatrixXd::Zero(3, num_points);
    for (int i = 0; i < num_points; ++i) {
        object_matrix.col(i) = points3D[i];
    }
    
    // Compute object vectors relative to first point (3xN)
    Eigen::MatrixXd object_vectors = object_matrix.colwise() - object_matrix.col(0);
    
    // Compute pseudo-inverse of object vectors (Nx3)
    Eigen::MatrixXd object_pinv = object_vectors.completeOrthogonalDecomposition().pseudoInverse();
    
    // Initialize image points (2xN)
    Eigen::MatrixXd image_vectors = Eigen::MatrixXd::Zero(2, num_points);
    Eigen::MatrixXd old_sop_image_points = Eigen::MatrixXd::Zero(2, num_points);
    
    for (int i = 0; i < num_points; ++i) {
        old_sop_image_points.col(i) = points2D[i];
    }

    const int MAX_ITERATIONS = 20;
    const double CONVERGENCE_THRESHOLD = 1.0;
    
    int count = 0;
    bool converged = false;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    double image_difference = std::numeric_limits<double>::max();

    while (!converged && count < MAX_ITERATIONS) {
        if (count == 0) {
            // Initialize image_vectors (2xN)
            for (int i = 0; i < num_points; ++i) {
                image_vectors.col(i) = points2D[i] - points2D[0];
            }
        } else {
            if (std::abs(translation(2)) < 1e-10) {
                std::cout << "Warning: Z translation too close to zero" << std::endl;
                return false;
            }
            
            // Compute scale factors for image points
            Eigen::VectorXd diff = (object_vectors.transpose() * rotation.row(2).transpose()) / translation(2);
            Eigen::MatrixXd sop_image_points = Eigen::MatrixXd::Zero(2, num_points);
            
            for (int i = 0; i < num_points; ++i) {
                sop_image_points.col(i) = points2D[i] + Eigen::Vector2d(diff(i), diff(i));
            }
            
            image_difference = (sop_image_points - old_sop_image_points).norm();
            if (std::isnan(image_difference)) {
                std::cout << "Warning: Numerical instability detected" << std::endl;
                return false;
            }
            
            old_sop_image_points = sop_image_points;
            image_vectors = sop_image_points.colwise() - sop_image_points.col(0);
        }

        // transformed_vectors should be 3x2
        Eigen::MatrixXd transformed_vectors = object_pinv.transpose() * image_vectors.transpose();
        
        // Extract 3D vectors
        Eigen::Vector3d ivect = transformed_vectors.col(0);
        Eigen::Vector3d jvect = transformed_vectors.col(1);
        
        // Debug output
        std::cout << "\nDebug values:" << std::endl;
        std::cout << "image_vectors shape: " << image_vectors.rows() << "x" << image_vectors.cols() << std::endl;
        std::cout << "object_pinv shape: " << object_pinv.rows() << "x" << object_pinv.cols() << std::endl;
        std::cout << "transformed_vectors shape: " << transformed_vectors.rows() << "x" << transformed_vectors.cols() << std::endl;
        std::cout << "transformed_vectors:\n" << transformed_vectors << std::endl;
        std::cout << "ivect: " << ivect.transpose() << std::endl;
        std::cout << "jvect: " << jvect.transpose() << std::endl;
        
        double i_square = ivect.dot(ivect);
        double j_square = jvect.dot(jvect);
        double ij = ivect.dot(jvect);
        
        std::cout << "i_square: " << i_square << " j_square: " << j_square << " ij: " << ij << std::endl;
        
        if (i_square < 0 || j_square < 0) {
            std::cout << "Warning: Invalid scale factors (negative values)" << std::endl;
            return false;
        }
        
        double scale1 = std::sqrt(i_square);
        double scale2 = std::sqrt(j_square);
                
        if (scale1 < 1e-6 || scale2 < 1e-6) {
            std::cout << "Warning: Scale factors too close to zero (< 1e-6)" << std::endl;
            return false;
        }
        
        Eigen::Vector3d row1 = ivect / scale1;
        Eigen::Vector3d row2 = jvect / scale2;
        Eigen::Vector3d row3 = row1.cross(row2);
        
        rotation.row(0) = row1;
        rotation.row(1) = row2;
        rotation.row(2) = row3;
        
        double scale = (scale1 + scale2) / 2.0;
        translation = Eigen::Vector3d(points2D[0].x(), points2D[0].y(), 1.0) / scale;
        
        converged = (count > 0) && (image_difference < CONVERGENCE_THRESHOLD);
        count++;
        std::cout << "Iteration: " << count << " Image difference: " << image_difference << std::endl;
    }
    
    if (count >= MAX_ITERATIONS) {
        std::cout << "Warning: Maximum iterations reached without convergence" << std::endl;
        return false;
    }
    
    proj_matrix->block<3, 3>(0, 0) = rotation;
    proj_matrix->col(3) = translation;
    
    return true;
}

void POSITEstimator::Residuals(const std::vector<X_t>& points2D,
                              const std::vector<Y_t>& points3D,
                              const M_t& proj_matrix, std::vector<double>* residuals) {
    residuals->clear();
    colmap::ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}