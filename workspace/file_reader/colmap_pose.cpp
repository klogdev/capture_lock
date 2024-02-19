#include <Eigen/Core>

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm> 

#include "base/projection.h"
#include "file_reader/kitti_odo_helper.h"

void ExtrinsicFromColmap(const std::string base_path, 
                         std::vector<std::vector<double>>& cali_info,
                         int img_begin, int img_end) {
    const int head = 4; // Fixed number of header lines to skip
    std::vector<std::vector<double>> trajectory; // raw "traj" to process all lines
    std::ifstream file(base_path);
    std::string line;

    // Skip header lines
    for (int i = 0; i < head && std::getline(file, line); i++) {}

    // Process lines for trajectory
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::vector<double> row;

        // Extract coordinates
        for (int i = 0; i < 8; ++i) {
            iss >> token;
            row.push_back(std::stod(token));
        }

        iss >> token; // skip the camera id

        // Extract image ID
        iss >> token; // Assuming the first token is the image ID
        int image_id = std::stoi(token.substr(img_begin, img_end - img_begin));
        row.push_back(static_cast<double>(image_id));

        trajectory.push_back(row);

        // std::cout << "current processed line is: " << std::endl;
        // for(const double entry: row){
        //     std::cout << entry << std::endl;
        // }
        // Skip the next line containing 2D pixel info
        std::getline(file, line);
    }

    // Sort the trajectory based on image ID (now the first element of each row)
    std::sort(trajectory.begin(), trajectory.end(), [](const std::vector<double>& a, const std::vector<double>& b) {
        return a.back() < b.back(); 
    });

    // Convert trajectory data to exclude image ID and image name for the output
    for (auto& vec : trajectory) {
        // Skip first element (image ID) and last element (image name), extract subvector [1:8]
        std::vector<double> coords(vec.begin() + 1, vec.begin() + 8); 
        cali_info.push_back(coords);
    }
}
