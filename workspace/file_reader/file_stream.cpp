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

void COLMAPStream(std::vector<std::string>& file_list, const std::string folder_dir,
                  int start, int end) {

    // Define ranges for the files you are looking for
    std::pair<int, int> index_range = {start, end};

    for (int index = index_range.first; index <= index_range.second; index++) {
        const std::string image_name = absl::StrFormat("P118%04d.JPG", index);
        const std::string image_path = absl::StrFormat("%s/%s", folder_dir, image_name);

        cv::Mat image = cv::imread(image_path);
        if (image.data != nullptr) {
            file_list.push_back(image_path);
        }
    }

}


void KITTIStream(std::vector<std::string>& file_list, const std::string folder_dir, 
                  int start, int end) {
    // Define ranges for the files you are looking for
    std::pair<int, int> index_range = {start, end};

    for (int index = index_range.first; index <= index_range.second; index++) {
        const std::string image_name = absl::StrFormat("%06d.PNG", index);
        const std::string image_path = absl::StrFormat("%s/%s", folder_dir, image_name);

        cv::Mat image = cv::imread(image_path);
        if (image.data != nullptr) {
            file_list.push_back(image_path);
        }
    }
}