#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

#include <absl/random/random.h>
#include <absl/strings/str_format.h>

/**
 * @brief load list of colmap images as a stream
*/
std::vector<std::vector<std::string>> COLMAPStream(const std::string folder_dir);