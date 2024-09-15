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
 */
void GetSortedFiles(const boost::filesystem::path& directory, 
                    std::vector<boost::filesystem::path>& files);

/**
 * @brief get associated rgb and depth map by providing
 * 2 parent directory
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
 * @brief process one pair of rgb and depth file
 */
void OnePairDepthRGB(std::string& rgb_file, std::string& depth_file,
                     Eigen::Vector3d& camera_pt, TUMIntrinsic& paras);


/**
 * @brief load aligned ground truth poses of TUM-RGBD file
 */
void LoadTUMPoses(std::string& gt_file, std::vector<Eigen::Vector4d>& quat,
                  std::vector<Eigen::Vector3d>& trans);