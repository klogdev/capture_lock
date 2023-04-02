#include <Eigen/Core>
#include "optim/ransac.h"

Eigen::Vector3d EpipoleEstimator::Estimate(const Eigen::Vector2d& point_2d_01,
                                           const Eigen::Vector2d& point_2d_02,
                                           const Eigen::Vector2d& point_2d_11,
                                           const Eigen::Vector2d& point_2d_12){
    Eigen::Vector3d homo01 = Point2dToHomo(point_2d_01);
    Eigen::Vector3d homo02 = Point2dToHomo(point_2d_02);
    Eigen::Vector3d homo11 = Point2dToHomo(point_2d_11);
    Eigen::Vector3d homo12 = Point2dToHomo(point_2d_12);

    Eigen::Vector3d line1 = PointToLine(homo01, homo11);
    Eigen::Vector3d line2 = PointToLine(homo02, homo12);

    return LineToPoint(line1, line2);
}

void EpipoleEstimator::Residual(std::vector<Eigen::Vector2d>& points1, 
                                std::vector<Eigen::Vector2d>& points2,
                                EigenVector3d& model, 
                                std::vector<double>* residuals){
    int n = points1.size();

    for(int i = 0; i < n; i++){
        
    }
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