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
    if (points2D.size() < 2 || points3D.size() < 2 || points2D.size() != points3D.size()) {
        throw std::runtime_error("Invalid input data.");
    }

    // Calculate object matrix
    Eigen::MatrixX3d pointsMatrix(points3D.size(), 3);

    // Fill the matrix with object points
    for (int i = 0; i < points3D.size(); ++i) {
        pointsMatrix.row(i) = points3D[i].transpose();
    }

    // Compute the pseudoinverse of the matrix
    Eigen::Matrix3d objectMatrix = (pointsMatrix.transpose() * pointsMatrix).inverse() * pointsMatrix.transpose();

    // Initialize variables
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    double scale;

    // Compute initial vectors from the first point to the others in both spaces
    std::vector<Eigen::Vector3d> objectVectors, imageVectors;
    for (size_t i = 1; i < points3D.size(); ++i) {
        objectVectors.push_back(points3D[i] - points3D[0]);
        // Convert 2D points to 3D by appending 1 for homogeneous coordinates
        imageVectors.push_back(Eigen::Vector3d(points2D[i][0], points2D[i][1], 1.0));
    }

    // Convert the first 2D point to 3D for homogeneous coordinates
    Eigen::Vector3d firstImagePoint(points2D[0][0], points2D[0][1], 1.0);
    translation = firstImagePoint; // Simplified initialization

    // Iterative process
    bool converged = false;
    double delta = 0.0;
    const double threshold = 0.01;
    int iteration = 0;
    const int max_iterations = 100;  // Add maximum iterations


    std::cout << "Starting POSIT iterations..." << std::endl;

    while (!converged && iteration < max_iterations) {
        iteration++;

        // Calculate rotation assuming objectMatrix is defined
        Eigen::Matrix3d objectMatrix; // Define or initialize objectMatrix appropriately
        rotation.col(0) = objectMatrix * imageVectors[0].normalized();
        rotation.col(1) = objectMatrix * imageVectors[1].normalized();
        rotation.col(2) = rotation.col(0).cross(rotation.col(1));

        // Normalize the columns of the rotation matrix
        rotation.col(0).normalize();
        rotation.col(1).normalize();
        rotation.col(2).normalize();

        // Compute new translation and scale
        scale = (rotation.col(0).norm() + rotation.col(1).norm()) / 2.0;
        translation = firstImagePoint / scale;

        // Check for convergence
        delta = (rotation * objectVectors[0] + translation - firstImagePoint).norm();

        std::cout << "Iteration " << iteration << ": delta = " << delta << std::endl;

        if (delta < threshold) {
            converged = true;
        }
    }

    if(!converged) {
        std::cout << "POSIT estimation failed to converge." << std::endl;
    }

    // Compose final pose
    proj_matrix->block<3,3>(0,0) = rotation;
    proj_matrix->block<3,1>(0,3) = translation;

    return true;
}

void POSITEstimator::Residuals(const std::vector<X_t>& points2D,
                              const std::vector<Y_t>& points3D,
                              const M_t& proj_matrix, std::vector<double>* residuals) {
    residuals->clear();
    colmap::ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}