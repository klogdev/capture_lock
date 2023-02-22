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
    for (int index = 150; index <= 340; ++index) {
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