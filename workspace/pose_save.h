#include <iostream>
#include <fstream>
#include <unordered_map>
#include <Eigen/Core>
#include <string>

#include "base/image.h"

void SavePoseToTxt(std::string outfile, std::unordered_map<int,colmap::Image>& global_image_map);