#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

#include <absl/random/random.h>
#include <absl/strings/str_format.h>

/**
 * @brief load list of colmap image paths as a stream
*/
void COLMAPStream(std::vector<std::string>& file_list, const std::string folder_dir, int start, int end);

/**
 * @brief load list of kitti gray image paths as a stream
*/
void KITTIStream(std::vector<std::string>& file_list, const std::string folder_dir, int start, int end);