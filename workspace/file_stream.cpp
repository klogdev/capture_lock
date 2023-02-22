#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>
#include <sys/stat.h>

#include <absl/strings/str_format.h>
#include "file_stream.h"

std::vector<std::string> FilePathStream(std::string folder_dir){
    std::vector<std::string> file_list;
    for (int index = 150; index <= 340; ++index) {
        std::string image_name = absl::StrFormat("P118%04d.JPG", index);
        std::string image_path =
        absl::StrFormat("%s/%s", folder_dir, image_name);

        struct stat sb;
        if (stat(image_path, &sb) == 0 && !(sb.st_mode & S_IFDIR)){
            file_list.push_back(image_path);
        }
    }
    return file_list;
}