#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

#include "base/projection.h"

std::vector<std::vector<double>> IntrinsicFromKittiCali(const std::string base_path, const std::string seq_num){
    std::vector<std::vector<double>> cali_info;
    std::string seq_path = base_path + seq_num;
    std::ifstream calis(seq_path);
    std::string line;

    while(std::getline(calis, line)){
        std::istringstream iss(line);
        std::vector<std::string> curr_line{std::istream_iterator<std::string>{iss}, 
                                           std::istream_iterator<std::string>{}};
        std::vector<double> curr_cali;
        for(auto s: curr_line){
            curr_cali.push_back(std::stod(s));
        }
        cali_info.push_back(curr_cali);
    }
    return cali_info;
}

Eigen::Matrix3d ProjMatFromCali(const std::vector<std::vector<double>> calib_vec,
                                  const int sensor_num){
    std::vector<double> curr_sensor = calib_vec[sensor_num];
    Eigen::Map<Eigen::Matrix<double, 3, 4>> proj_mat(curr_sensor.data());

    Eigen::Matrix3d K;
    Eigen::Matrix3d R; 
    Eigen::Vector3d T;

    bool decompose = colmap::DecomposeProjectionMatrix(proj_mat, &K, &R, &T);

    return K;
}