#include <Eigen/Core>
#include "base/camera.h"

double ReprojErr(Eigen::Vector2d& point_2d, Eigen::Vector3d& point_3d, 
                 colmap::Camera camera, Eigen:Matrix3x4d& proj_matrix){

        Eigen::Vector4d point_3d_homo = Eigen::Vector4d::Identity();
        point_3d_homo.topRows(3) = point_3d; 

        Eigen::Matrix3d calibration = camera.CalibrationMatrix();
        Eigen::Vector3d projected_2d = calibration*proj_matrix*point_3d_homo;

        return std::abs(pow(point_2d[0] - projected_2d[0]/projected_2d[2], 2) + pow(point_2d[1] - projected_2d[1]/projected_2d[2], 2));
}