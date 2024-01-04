#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

#include "base/projection.h"

void IntrinsicFromKittiCali(const std::string base_path, const std::string seq_num,
                            std::vector<std::vector<double>>& cali_infos){
    std::string seq_path = base_path + seq_num + "/calib.txt";
    std::ifstream calis(seq_path);
    std::string line;
    
    while(std::getline(calis, line)){
        std::istringstream iss(line);
        // the first iterator process the "line", the second is the dummy one as
        // an end iterator
        std::vector<std::string> curr_line{std::istream_iterator<std::string>{iss}, 
                                           std::istream_iterator<std::string>{}};

        // need to skip the name of the sensor, i.e. P0
        curr_line.erase(curr_line.begin(), curr_line.begin() + 1);

        std::vector<double> curr_cali;
        for(auto s: curr_line){
            curr_cali.push_back(std::stod(s));
        }
        cali_infos.push_back(curr_cali);
    }
}

void ExtrinsicFromKitti(const std::string base_path, const std::string seq_num,
                        std::vector<std::vector<double>>& pose_list){
    std::string pose_path = base_path + seq_num;
    std::ifstream calis(pose_path);
    std::string line;

    while(std::getline(calis, line)){
        std::istringstream iss(line);
        // the first iterator process the "line", the second is the dummy one as
        // an end iterator
        std::vector<std::string> curr_line{std::istream_iterator<std::string>{iss}, 
                                           std::istream_iterator<std::string>{}};

        std::vector<double> curr_cali;
        for(auto s: curr_line){
            curr_cali.push_back(std::stod(s));
        }
        pose_list.push_back(curr_cali);
    }
}

void GetCameraModelParams(std::vector<double>& row_major_cali, 
                          std::vector<double>& simple_params){
    // we assume the simple pinhole model's params has the order: f, cx, cy
    simple_params.push_back(row_major_cali[0]);
    simple_params.push_back(row_major_cali[2]);
    simple_params.push_back(row_major_cali[6]);
}