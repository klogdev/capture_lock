#ifndef UTIL__FILE_SAVE_H_
#define UTIL__FILE_SAVE_H_

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <Eigen/Core>
#include <string>
#include <filesystem>

/**
 * @brief save the 1D vector to a text file
 * can be applied to averaged residuals, Frobenius norm et.al.
*/
template<typename T>
void save1DVec(const std::vector<T>& data_vec, std::string out_path) {
    std::filesystem::path path(out_path);
    std::filesystem::path directory = path.parent_path();

    if(!std::filesystem::exists(directory)) {
        try {
            std::filesystem::create_directory(directory);
            std::cout << "created directory: " << directory << std::endl;
        }
        catch(const std::filesystem::filesystem_error& e) {
            std::cerr << "failed to created the directory: " << e.what() << std::endl;
        }
    }

    std::ofstream outfile(out_path, std::ios::app);
    if (!outfile.is_open()) {
        std::cerr << "Error: Unable to open file " << out_path << " for writing." << std::endl;
        return;
    }

    // each value as a single line
    for (const auto& element : data_vec) {
        outfile << element << "\n";
    }

    outfile.close();
    std::cout << "1D vector data saved to file: " << out_path << std::endl;
}

/**
 * @brief save the 2D vector to a text file
 * can be applied to list of residuals
*/
void save2DdoubleVec(const std::vector<std::vector<double>>& data_vec, std::string out_path);

#endif // UTIL__FILE_SAVE_H_
