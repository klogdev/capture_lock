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

std::vector<GyroData> LoadGyroData(string file_path)

std::vector<std::string> FilePathStream(const std::string folder_dir);