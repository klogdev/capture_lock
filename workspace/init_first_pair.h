#include <string>
#include <Eigen/Core>
#include "optim/ransac.h"
#include "estimators/pose.h"
#include "base/image.h"
#include "base/point3d.h"
#include "base/camera.h"
#include "feature/sift.h"
#include "feature/image_sift.h"


void InitFirstPair(const std::string first_path, const std::string second_path,
                    colmap::Camera& camera,
                    std::unordered_map<int,colmap::Image>& global_image_map,
                    std::unordered_map<int,std::vector<sift::Keypoint>>& global_keypts_map,
                    std::unordered_map<int,colmap::Point3D>& global_3d_map,
                    int resize_w, int resize_h);