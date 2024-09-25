#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <string>
#include <filesystem>


void save2DdoubleVec(const std::vector<std::vector<double>>& data_vec, std::string out_path) {
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

    std::ofstream outfile(out_path);
    if (!outfile.is_open()) {
        std::cerr << "Error: Unable to open file " << out_path << " for writing." << std::endl;
        return;
    }

    for (const auto& row : data_vec) {
        for (size_t i = 0; i < row.size(); ++i) {
            outfile << row[i];
            if (i != row.size() - 1) {
                outfile << " "; // Separate elements within a row with space
            }
        }
        outfile << "\n"; // Move to the next line after each row
    }

    outfile.close();
    std::cout << "2D vector data saved to file: " << out_path << std::endl;
}

void Save3DPoints(const std::string& filename, const std::vector<std::vector<Eigen::Vector3d>>& points3D) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }

    for (const auto& vector : points3D) {
        for (const Eigen::Vector3d& point : vector) {
            file << point.x() << " " << point.y() << " " << point.z() << "\n";
        }
        // You may want to separate each vector's points for clarity
        file << "\n";  // Blank line to separate different vectors of points
    }

    file.close();
    std::cout << "3D point data saved to file: " << filename << std::endl;
}
