#include <Eigen/Core>
#include <string>
#include <map>

#include "feature/sift.h"
#include "feature/image_sift.h"
#include "base/image.h"
#include "base/point3d.h"
#include "base/camera.h"
#include "base/track.h"

/**
 * @brief main fn to incrementally
 * register new image frames
 * three global maps should have consistent ID
*/
void IncrementOneImage(std::string image_path,int new_id,
                       int last_id, colmap::Camera& camera,
                       std::unordered_map<int,colmap::Image>& global_image_map,
                       std::unordered_map<int,std::vector<sift::Keypoint>>& global_keypts_map,
                       std::unordered_map<int,colmap::Point3D>& global_3d_map,
                       std::map<int, std::pair<Eigen::Vector4d,Eigen::Vector3d>> gt_map,
                       int resize_w, int resize_h);


    