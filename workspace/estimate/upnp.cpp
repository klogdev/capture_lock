#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

#include "estimators/utils.h"
#include "estimate/upnp.h"

std::vector<UPnPEstimator::M_t> UPnPEstimator::Estimate(
    const std::vector<X_t>& points2D, const std::vector<Y_t>& points3D) {
    std::vector<M_t> models;

    if(points2D.size() < kMinNumSamples || points3D.size() < kMinNumSamples) {
        return models;
    }

    // convert points to OpenCV's type
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;

    for(size_t i = 0; i < points2D.size(); i++) {
        objectPoints.emplace_back(points3D[i].x(), points3D[i].y(), points3D[i].z());
        imagePoints.emplace_back(points2D[i].x(), points2D[i].y());
    }

    // Provide an identity matrix for the camera matrix
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs;  // Empty distortion coefficients (no distortion assumed)

    cv::Mat rvec, tvec;

    bool success = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_UPNP);

    if(success) {
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        M_t model;
        model.block<3, 3>(0, 0) = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R.ptr<double>());
        model.col(3) = Eigen::Map<Eigen::Vector3d>(tvec.ptr<double>());

        models.push_back(model);
    }

    return models;
}

void UPnPEstimator::Residuals(const std::vector<X_t>& points2D,
                             const std::vector<Y_t>& points3D,
                             const M_t& proj_matrix, std::vector<double>* residuals) {
    residuals->clear();
    colmap::ComputeSquaredReprojectionError(points2D, points3D, proj_matrix, residuals);
}