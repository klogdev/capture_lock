#ifndef FILE_READER_TUM_RGBD_H_
#define FILE_READER_TUM_RGBD_H_

#include <boost/filesystem.hpp>
#include <Eigen/Core>

#include "base/camera.h"
#include "base/camera_models.h"
#include "base/image.h"
#include "base/point3d.h"

#include "optim/bundle_adjustment.h"

#include "feature/sift.h"

/**
 * @brief assign precalibrated intrinsic paramaters
 * check https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
 */
class TUMIntrinsic {
public:
    TUMIntrinsic(int sequence_num = 1) {
        switch(sequence_num) {
            case 1:  // fr1
                fx = 517.3; fy = 516.5;
                cx = 318.6; cy = 255.3;
                scale = 5000.0;
                break;
            case 2:  // fr2
                fx = 520.9; fy = 521.0;
                cx = 325.1; cy = 249.7;
                scale = 5208.0;
                break;
            case 3: // euroc mav, v01, stereo
                fx = 435.2; fy = 435.2;
                cx = 367.5; cy = 252.2;
                break;
            default:
                std::cerr << "Warning: Unknown sequence number " << sequence_num 
                         << ". Using fr1 parameters." << std::endl;
                fx = 517.3; fy = 516.5;
                cx = 318.6; cy = 255.3;
                scale = 5000.0;
        }
    }

    double fx;  // Focal length in x
    double fy;  // Focal length in y
    double cx;  // Optical center x
    double cy;  // Optical center y
    double scale; // scale constant for depth
};

/**
 * @brief get sorted files via timestamps
 * @arg directory: parent path
 * files: all files inside the parent path
 */
void GetSortedFiles(const boost::filesystem::path& directory, 
                    std::vector<boost::filesystem::path>& files);

/**
 * @brief get associated rgb and depth map by providing
 * 2 parent directory, all_rgb & all_depth should be aligned by there timestamp
 */
void DepthRGBMap(const boost::filesystem::path& rgb_path,
                 const boost::filesystem::path& depth_path,
                 std::vector<boost::filesystem::path>& all_rgb,
                 std::vector<boost::filesystem::path>& all_depth);

/**
 * @brief convert pixel&depth value to camera space points
 */
void DepthToCameraSpace(int u, int v, float depth, Eigen::Vector3d& point,
                        TUMIntrinsic& paras);

/**
 * @brief process one pair of rgb and depth file, with 
 * outputting point cloud inside the camera space
 * @arg normalized_pts: normalized pts from depth map that has non-zero depth
 * @note when using SIFT, switch cv::Mat to sift::Keypoint
 */
void OnePairDepthRGB(const std::string& image_file, 
                     const std::string& depth_file, 
                     const int curr_idx,
                     std::vector<Eigen::Vector3d>& camera_pts, 
                     std::vector<Eigen::Vector2d>& normalized_pts, 
                     std::unordered_map<int, std::vector<sift::Keypoint>>& global_keypts_map,
                     TUMIntrinsic& paras);

/**
 * @brief load pre-aligned ground truth poses of TUM-RGBD file
 * the file was preprocessed via a separate python script, to aligning 
 * depth/image with g.t. poses
 * the format is: time,tx,ty,tz,qx,qy,qz,qw
 */
void LoadTUMPoses(std::string& gt_file, std::vector<Eigen::Vector4d>& quat,
                  std::vector<Eigen::Vector3d>& trans);

/**
 * @brief convert a set of camera points to world space, i.e. 
 * can be a set from a single TUM depth map
 */
void PairsCameraToWorld(const std::vector<Eigen::Vector3d>& camera_pts,
                        const Eigen::Vector4d& quat,
                        const Eigen::Vector3d& trans,
                        std::vector<Eigen::Vector3d>& world_pts);


/**
 * covert all pairs of depth maps and g.t. poses to world space points
 * @arg depth_files: a list of directory to depth map files
 * gt_pose: a single files with pre-aligned gt poses 
 */
void ProcessAllPairs(const std::vector<std::string>& image_files,
                     const std::vector<std::string>& depth_files,
                     const std::string& gt_pose,
                     TUMIntrinsic& paras,
                     std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                     std::vector<std::vector<Eigen::Vector3d>>& points3D,
                     std::vector<Eigen::Matrix<double, 3, 4>>& composed_extrinsic);

/**
 * @brief set a virtual colmap camera for colmap image 
 * declaration and bundle adjustment
 */
void SetVirtualColmapCamera(colmap::Camera& virtual_camera);

/**
 * @brief all current 2d points with it's default id, and register 
 * the corresponded 3d point
 */
void SetPoint3dOneImage(colmap::Image* curr_img,
                        colmap::Image* last_img,
                        colmap::Camera& camera,
                        std::vector<Eigen::Vector3d>& point_3d,
                        std::unordered_map<int, colmap::Point3D>& global_3d_map,
                        std::unordered_map<int, std::vector<sift::Keypoint>>& global_keypts_map,
                        int& curr_3d_idx);

void PrintParameterBlocks(ceres::Problem& problem);

/**
 * @brief special Bundle Adjustment for TUM RGBD, with fixed poses
 * only optimize 
 */
void TUMBundle(std::unordered_map<int, colmap::Image*>& global_img_map,
               std::unordered_map<int, colmap::Point3D>& global_3d_map,
               colmap::Camera& camera, double anchor_weight = 0.0);


/**
 * @brief set options for bundle adjustment
 * here we aim to fix g.t. poses but only polish 3d points which 
 * init from depth maps
 */
void SetBAOptions(colmap::BundleAdjustmentOptions& ba_options);

/**
 * @brief retrieve optimized 3d points from a single colmap image
 */
void RetrievePairsfromImage(colmap::Image* curr_img, 
                           std::unordered_map<int, colmap::Point3D>& global_3d_map,
                           std::vector<Eigen::Vector2d>& point2ds,
                           std::vector<Eigen::Vector3d>& point3ds);

#endif // FILE_READER_TUM_RGBD_H_