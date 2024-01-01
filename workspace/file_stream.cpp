#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>

#include <absl/random/random.h>
#include <absl/strings/str_format.h>
#include "file_stream.h"

#include <vector>
#include <string>
#include <absl/strings/str_format.h>
#include <opencv2/opencv.hpp>

std::vector<std::vector<std::string>> FilePathStream(const std::string folder_dir) {
    std::vector<std::vector<std::string>> file_list;

    // Define ranges for the files you are looking for
    std::vector<std::pair<int, int>> index_ranges = {{141, 200},{305,347}};

    for (const auto& range : index_ranges) {
        std::vector<std::string> curr_list;
        for (int index = range.first; index <= range.second; ++index) {
            const std::string image_name = absl::StrFormat("P118%04d.JPG", index);
            const std::string image_path = absl::StrFormat("%s/%s", folder_dir, image_name);

            cv::Mat image = cv::imread(image_path);
            if (image.data != nullptr) {
                curr_list.push_back(image_path);
            }
        }
        file_list.push_back(curr_list);
    }
    return file_list;
}


