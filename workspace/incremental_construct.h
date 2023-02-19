#include <Eigen/Core>
#include "feature/sift.h"
#include "feature/image_sift.h"
#include "base/image.h"
#include "base/point3d.h"
#include <string>
#include "feature/sift.h"

Image LoadImage(const std::string folder, const std::string image_name);

colmap::Image SIFTtoCOLMAPImage(int image_id, std::vector<Eigen::Vector2d> features,
                                const& colmap::Camera camera);

std::vector<colmap::Point3D> IncrementOneImage(std::string image_path,
                                            int next_id,
                                            const& colmap::Image last_image,
                                            const& colmap::Camera camera,
                                            std::unordered_map<int,colmap::Image>& global_image_map,
                                            std::unordered_map<int,Eigen::Vector3d>& global_keypts_map);

//generate sift keypoints from sift image
std::vector<sift::Keypoint> GetKeyPoints(Image& image);

//get a hash map of <tuple,sift::keypoint>, use (i,j) pair to connect
//with Point2D in colmap::Image
std::unordered_map<std::pair<int,int>, sift::Keypoint> SIFTPtsToHash(std::vector<sift::Keypoint> key_points);

//convert i,j coord from SIFT points to eigen vector for the init of colmap::Image
std::vector<Eigen::Vector2d> SIFTPtsToVec(std::vector<sift::Keypoint> key_points);

std::vector<Eigen::Vector3d> GetPoint3Dfrom2D(int image_id, 
                                            std::vector& global_image_map);

    