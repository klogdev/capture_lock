#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <string>


void save2DdoubleVec(const std::vector<std::vector<double>>& data_vec, std::string out_path) {
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
