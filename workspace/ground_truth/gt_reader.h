#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

Eigen::Matrix3x4d ExtrinsicFromGT(const std::vector<double>& gt_one_row);

std::vector<Eigen::Matrix3x4d> GetAllExtrinsic(const std::string base_path,
                                               const std::string seq_num);