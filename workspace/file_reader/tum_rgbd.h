#include <boost/filesystem.hpp>
#include <Eigen/Core>

/**
 * @brief assign precalibrated intrinsic paramaters
 * check https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
 */
struct TUMIntrinsic {
    double fx = 525.0;  // Focal length in x
    double fy = 525.0;  
    double cx = 319.5;  // Optical center x
    double cy = 239.5; 
    double scale = 5000.0; // scale constant for scale
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
 */
void OnePairDepthRGB(const std::string& depth_file, std::vector<Eigen::Vector3d>& camera_pts, 
                     std::vector<Eigen::Vector2d>& normalized_pts, TUMIntrinsic& paras);

/**
 * @brief load pre-aligned ground truth poses of TUM-RGBD file
 * the file was preprocessed via a separate python script, to aligning 
 * depth/image with g.t. poses
 * the format is: time, qx,qy,qz,qw,tx,ty,tz
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
 */
void ProcessAllPairs(const std::vector<std::string>& depth_files,
                     const std::vector<Eigen::Vector4d>& quats,
                     const std::vector<Eigen::Vector3d>& trans,
                     TUMIntrinsic& paras,
                     std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                     std::vector<std::vector<Eigen::Vector3d>>& points3D);