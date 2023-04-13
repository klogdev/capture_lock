#include "epipole_estimator.h"
#include <Eigen/Core>
#include "feature/sift.h"

Eigen::Vector3d FinalAlignment(std::vector<sift::Keypoint>& locked_keypts, 
                               std::string compensate_path);
