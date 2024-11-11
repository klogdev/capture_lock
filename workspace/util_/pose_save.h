#include <iostream>
#include <fstream>
#include <unordered_map>
#include <Eigen/Core>
#include <string>

#include "base/image.h"

/**
 * @brief save poses of the VO by using similar format as COLMAP data
*/
void SavePoseToTxt(std::string outfile, std::unordered_map<int,colmap::Image>& global_image_map);
