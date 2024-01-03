#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

#include <absl/random/random.h>
#include <absl/strings/str_format.h>

/**
 * @brief encapsulate the kitti raw imu's gyro data as a structure
*/
struct GyroData {
    double timestamp;
    double wx, wy, wz;
};

/**
 * @brief load a sequence timestamp (can be applied on IMU/Image sensors)
*/
std::vector<double> LoadTimeStamp(std::string timestamp);

/**
 * @brief load a gyro data from a single raw IMU reading, the line 17-19 (0 indexed)
 * are angular rate along x,y,z
*/
Eigen::Vector3d LoadOneGyro(std::string one_file_path);

/**
 * @brief load sequence of gyro data from stream of imu reading
*/
std::vector<GyroData> LoadGyroData(std::string timestamp_path, 
                                   std::string data_path,
                                   int start_frame,
                                   int end_frame);

/**
 * @brief generate path stream of all IMU reading with absl
*/
std::vector<std::string> GyroPathStream(const std::string folder_dir);

/**
 * @brief convert time string from KITTI raw data to a relative double value
 * a sample string could be: 2011-09-26 13:02:25.964389445
*/
double TimeStrToDouble(std::string timestamp);