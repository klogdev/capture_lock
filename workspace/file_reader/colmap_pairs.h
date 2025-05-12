#include <Eigen/Core>
#include <vector>

using Matrix3x4d = Eigen::Matrix<double, 3, 4>;

/**
 * @brief load colmap pairs, the files are preprocessed via a separate
 * python script; the patterns of the files are as follows:
 * 
 * pose_i: q0, q1, q2, q3, tx, ty, tz
 * u, v, x, y, z
 * .....
 * 
 * @param filename 
 * @param points2D 
 * @param points3D 
 * @param composed_extrinsic 
 */
void LoadColmapPairs(const std::string& filename,
                     std::vector<std::vector<Eigen::Vector2d>>& points2D,
                     std::vector<std::vector<Eigen::Vector3d>>& points3D,
                     std::vector<Matrix3x4d>& composed_extrinsic);

void printData(const std::vector<std::vector<Eigen::Vector2d>>& points2D,
               const std::vector<std::vector<Eigen::Vector3d>>& points3D,
               const std::vector<Matrix3x4d>& composed_extrinsic);

/**
 * @brief filter the points by normalized radius
 * 
 * @param all_points2D 
 * @param all_points3D 
 * @param filtered_points2D 
 * @param filtered_points3D 
 * @param radius 
 */
void FilterByNormalizedRadius(const std::vector<Eigen::Vector2d>& all_points2D,
                              const std::vector<Eigen::Vector3d>& all_points3D,
                              std::vector<Eigen::Vector2d>& filtered_points2D,
                              std::vector<Eigen::Vector3d>& filtered_points3D,
                              double radius);