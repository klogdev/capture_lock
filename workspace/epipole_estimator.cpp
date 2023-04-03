#include <Eigen/Core>
#include "optim/ransac.h"

#include "epipole_estimator.h"

Eigen::Vector3d EpipoleEstimator::Estimate(const std::vector<X_t>& pair1,
                                           const std::vector<Y_t>& pair2){
    Eigen::Vector3d homo01 = Point2dToHomo(pair1[0].point_lock);
    Eigen::Vector3d homo02 = Point2dToHomo(pair2[0].point_lock);
    Eigen::Vector3d homo11 = Point2dToHomo(pair1[0].point_comp);
    Eigen::Vector3d homo12 = Point2dToHomo(pair2[0].point_comp);

    Eigen::Vector3d line1 = PointToLine(homo01, homo11);
    Eigen::Vector3d line2 = PointToLine(homo02, homo12);

    return LineToPoint(line1, line2);
}

void EpipoleEstimator::Residuals(const std::vector<X_t>& pairs1, 
                                 const std::vector<Y_t>& pairs2,
                                 const M_t& model, 
                                 std::vector<double>* residuals){
    int n = points1.size();

    for(int i = 0; i < n; i++){
        Eigen::Vector3d homo01 = Point2dToHomo(pairs1[i].point_lock);
        Eigen::Vector3d homo02 = Point2dToHomo(pairs2[i].point_lock);
        Eigen::Vector3d homo11 = Point2dToHomo(pairs1[i].point_comp);
        Eigen::Vector3d homo12 = Point2dToHomo(pairs2[i].point_comp);

        Eigen::Vector3d line1 = PointToLine(homo01, homo11);
        Eigen::Vector3d line2 = PointToLine(homo02, homo12);

        Eigen::Vector3d curr_intersection = LineToPoint(line1, line2);
        (*residuals)[i] = SquareError(model, curr_intersection);
    }
}

bool EstimateEpipole(const colmap::RANSACOptions& ransac_options,
                     const std::vector<PixelPair>& pairs1
                     const std::vector<PixelPair>& pairs2
                     std::vector<char>* inlier_mask, Eigen::Vector3d* model){
    colmap::RANSAC<EpipoleEstimator> ransac(ransac_options);
    const auto report = ransac.Estimate(pairs1, pairs2);

    if (!report.success) {
        return false;
    }

    *inlier_mask = report.inlier_mask;
    *model = report.model;

    return report.success;
}

//helper functions
Eigen::Vector3d Point2dToHomo(Eigen::Vector2d point_2d){
    homo = Eigen::Vector4d::Identity();
    homo.topRows(2) = point_2d;
    return homo; 
}

Eigen::Vector3d PointToLine(Eigen::Vector3d pixel1, Eigen::Vector3d pixel2){
    return pixel1.cross(pixel2);
}

Eigen::Vector3d LineToPoint(Eigen::Vector3d line1, Eigen::Vector3d line2){
    return line1.cross(line2);
}

double SquareError(Eigen::Vector3d homo1, Eigen::Vector3d homo2){
    return std::abs(pow(homo1[0] - homo2[0], 2) + pow(homo1[1] - homo2[1], 2));
}