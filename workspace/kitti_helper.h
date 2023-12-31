#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

/**
 * @brief convert the KITTI Odometry data's calibration file 
 * to vectors of doubles for all sensors
 */
std::vector<std::vector<double>> IntrinsicFromKittiCali(const std::string base_path, const std::string seq_num);

/**
 * @brief decompose the calibration matrix to get the intrinsic one
*/
Eigen::Matrix3d ProjMatFromCali(const std::vector<std::vector<double>> calib_vec,
                                  const int sensor_num);