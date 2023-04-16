#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

//convert the KITTI's calibration file to vectors of doubles for all sensors
std::vector<std::vector<double>> IntrinsicFromKittiCali(const std::string base_path, const std::string seq_num);

//decompose the calibration matrix to get the intrinsic
Eigen::Matrix3d ProjMatFromCali(const std::vector<std::vector<double>> calib_vec,
                                  const int sensor_num);