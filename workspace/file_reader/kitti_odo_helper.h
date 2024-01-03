#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

/**
 * @brief convert the KITTI Odometry data's calibration file 
 * to vectors of doubles for all sensors
 * noticed we load all sensor's calis, i.e. P0 to P4, 
 * (two gray scale camera and two colored cameras)
 * the P0 is K*Identity matrix, and P1,2,3 need to transform to P0 first
 * but only do odometry for the first camera
 * @arg cali_infos: 4 cameras' calibration matrices
 */
void IntrinsicFromKittiCali(const std::string base_path, const std::string seq_num,
                            std::vector<std::vector<double>>& cali_infos);

/**
 * @brief read a single sequence of extrinsic matrices from G.T.
*/
void ExtrinsicFromKitti(const std::string base_path, const std::string seq_num,
                        std::vector<std::vector<double>>& pose_list);
