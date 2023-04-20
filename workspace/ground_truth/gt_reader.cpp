#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

#include <absl/random/random.h>
#include <absl/strings/str_format.h>

Eigen::Matrix3x4d ExtrinsicFromGT(const std::vector<double>& gt_one_row){
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> extrinsic(gt_one_row.data());

    std::cout << "extrinsic matrix: " << std::endl;
    std::cout << extrinsic << std::endl;
    return extrinsic;
}

std::vector<Eigen::Matrix3x4d> GetAllExtrinsic(const std::string base_path,
                                               const std::string seq_num){
    std::vector<Eigen::Matrix3x4d> pose_info;
    std::string seq_path = base_path + seq_num + ".txt";
    std::ifstream poses(seq_path);
    std::string line;

    while(std::getline(poses, line)){
        std::istringstream iss(line);
        std::vector<std::string> curr_line{std::istream_iterator<std::string>{iss}, 
                                           std::istream_iterator<std::string>{}};

        std::vector<double> curr_pose;
        for(auto s: curr_line){
            curr_pose.push_back(std::stod(s));
        }
        pose_info.push_back(ExtrinsicFromGT(curr_pose));
    }
    return pose_info;
}