#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>

#include <absl/random/random.h>
#include <absl/strings/str_format.h>
#include "file_stream.h"

std::vector<std::string> FilePathStream(const std::string folder_dir){
    std::vector<std::string> file_list;
    for (int index = 141; index <= 146; ++index) {
        const std::string image_name = absl::StrFormat("P118%04d.JPG", index);
        const std::string image_path =
        absl::StrFormat("%s/%s", folder_dir, image_name);

        cv::Mat image = cv::imread(image_path);
        if(image.data != nullptr){
            file_list.push_back(image_path);
        }
    }
    return file_list;
}

std::vector<std::string> LoadTimeStamp(std::string timestamp){
    std::vector<std::string> time_list;
    std::ifstream times(timestamp);
    std::string line;

    while(std::getline(times, line)){
        std::isstringstream iss(line);
        std::vector<std::string> curr_line{std::istream_iterator<std::string>{iss}, 
                                           std::istream_iterator<std::string>{}};

        time_list.push_back(curr_line[1]);
    }

    return time_list;
}

Eigen::Vector3d LoadOneGyro(std::string one_file_path){
    std::ifstream gyro(one_file_path);
    std::string line;

    while(std::getline(gyro, line)){
        std::isstringstream iss(line);
        std::vector<std::string> curr_line{std::istream_iterator<std::string>{iss}, 
                                           std::istream_iterator<std::string>{}};

        Eigen::Vector3d curr_gyro(std::stod(curr_line[17]),
                                  std::stod(curr_line[18]),
                                  std::stod(curr_line[19]));
    }
    return curr_gyro;
}

std::vector<GyroData> LoadGyroData(std::string timestamp_path, 
                                   std::string data_path){
    
}