#include <iostream> 
#include <string>
#include <fstream>

#include <Eigen/Core>

/**
 * @brief read a single sequence of extrinsic matrices from 
 * Colmap formatted G.T.
*/
void ExtrinsicFromColmap(const std::string base_path, std::vector<std::vector<double>>& cali_info,
                        int img_begin, int img_end); 