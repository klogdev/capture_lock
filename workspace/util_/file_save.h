#include <iostream>
#include <fstream>
#include <unordered_map>
#include <Eigen/Core>
#include <string>

/**
 * @brief save the 1D vector to a text file
 * can be applied to averaged residuals, Frobenius norm et.al.
*/
void save1DdoubleVec(const std::vector<double>& data_vec, std::string out_path);

/**
 * @brief save the 2D vector to a text file
 * can be applied to list of residuals
*/
void save2DdoubleVec(const std::vector<std::vector<double>>& data_vec, std::string out_path);