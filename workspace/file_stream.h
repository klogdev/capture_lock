#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

#include <absl/random/random.h>
#include <absl/strings/str_format.h>

struct GyroData {
    double timestamp;
    double wx, wy, wz;
};

std::vector<std::string> LoadTimeStamp(std::string timestamp);

Eigen::Vector3d LoadOneGyro(std::string one_file_path);

std::vector<GyroData> LoadGyroData(std::string timestamp_path, 
                                   std::string data_path);

std::vector<std::string> FilePathStream(const std::string folder_dir);