#include <Eigen/Core>
#include "feature/sift.h"
#include "feature/image_sift.h"
#include "base/image.h"
#include "base/point3d.h"
#include "base/camera.h"
#include "base/track.h"
#include <string>
#include "feature/sift.h"

colmap::Image SIFTtoCOLMAPImage(int image_id, std::vector<Eigen::Vector2d> features,
                                const colmap::Camera& camera);

//three global maps should have consistent ID
void IncrementOneImage(std::string image_path,int next_id,
                        colmap::Image& last_image,
                        colmap::Camera& camera,
                        std::unordered_map<int,colmap::Image>& global_image_map,
                        std::unordered_map<int,std::vector<sift::Keypoint>>& global_keypts_map,
                        std::unordered_map<int,Eigen::Vector3d>& global_3d_map,
                        colmap::Track global_track);

//generate sift keypoints from sift image
std::vector<sift::Keypoint> GetKeyPoints(Image& image);

//convert i,j coord from SIFT points to eigen vector for the init of colmap::Image
std::vector<Eigen::Vector2d> SIFTPtsToVec(std::vector<sift::Keypoint> key_points);

    