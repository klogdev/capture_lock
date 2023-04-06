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

std::vector<std::string> GyroPathStream(const std::string folder_dir){
    std::vector<std::string> file_list;
    for (int index = 1; index <= 77; ++index) {
        const std::string gyro_name = absl::StrFormat("%10d.txt", index);
        const std::string gyro_path =
        absl::StrFormat("%s/%s", folder_dir, gyro_name);
        file_list.push_back(gyro_path);
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
    //each gyro data only has 1 line with different var
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
                                   std::string data_path,
                                   int start_frame,
                                   int end_frame){
    std::vector<std::string> time_frames = LoadTimeStamp(timestamp_path);
    std::vector<std::string> gyro_paths = GyroPathStream(data_path);

    std::vector<GyroData> gyro_data;

    for(int i = start_frame; i <= end_frame; i++){
        curr_data = LoadOneGyro(gyro_paths[i]);
        GyroData curr_gyro;
        curr_gyro.timstamp = time_frames[i];
        curr_gyro.wx = curr_data(0);
        curr_gyro.wy = curr_data(1);
        curr_gyro.wz = curr_data(2);
        gyro_data.push_back(curr_gyro);
    }
    return gyro_data;
}