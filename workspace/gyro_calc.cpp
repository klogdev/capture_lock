#include "file_stream.h"
#include "gyro_calc.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Quaterniond EstimateGyroAccumulation(vector<GyroData> GyroData,
                                            int frame1, int frame2){
    double dt = GyroData[1].timestamp - GyroData[0].timestamp;
    double acc_x = 0.0, acc_y = 0.0, acc_z = 0.0;
    for (int i = frame1; i <= frame2; i++){
        double wx = GyroData[i].wx;
        double wy = GyroData[i].wy;
        double wz = GyroData[i].wz;
        acc_x += wx*dt;
        acc_y += wy*dt;
        acc_z += wz*dt;
    }
    std::cout << "Gyro Accumulation: " << acc_x << " " << acc_y << " " << acc_z << std::endl;

    //get quaternion
    Eigen::Vector3d dtheta;
    dtheta << acc_x, acc_y, acc_z;
    double dtheta_norm = dtheta.norm();
    Eigen::Quaterniond quat(cos_half_norm, sin_half_norm * dtheta / dtheta_norm);

    return quat;
}