#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

/**
 * @brief convert the KITTI Odometry data's calibration file 
 * to vectors of doubles for all sensors
 * noticed we load all sensor's calis, i.e. P0 to P4, but only
 * do odometry for the first camera
 */
std::vector<std::vector<double>> IntrinsicFromKittiCali(const std::string base_path, const std::string seq_num);

/**
 * @brief decompose the calibration matrix to get the intrinsic one
*/
Eigen::Matrix3d ProjMatFromCali(const std::vector<std::vector<double>> calib_vec,
                                  const int sensor_num);