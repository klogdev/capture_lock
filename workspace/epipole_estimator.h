#include <Eigen/Core>
#include <vector>

#include "optim/ransac.h"

class EpipoleEstimator{
    public: 
        
};

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