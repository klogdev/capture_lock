#include "file_stream.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

//could be converted to 4d vec if needed
Eigen::Quaterniond EstimateGyroAccumulation(std::vector<GyroData>& gyro_data,
                                         int frame1, int frame2);