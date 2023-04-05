#include "file_stream.h"
#include <Eigen/Core>

Eigen::Quaterniond EstimateGyroAccumulation(std::vector<GyroData>& gyro_data,
                                         int frame1, int frame2);