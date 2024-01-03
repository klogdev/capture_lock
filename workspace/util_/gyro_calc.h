#include "file_stream.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief This fn are use to load gyro/imu reading from KITTI raw
 * data (not odometry)
 */
Eigen::Quaterniond EstimateGyroAccumulation(std::vector<GyroData>& gyro_data,
                                            int frame1, int frame2);