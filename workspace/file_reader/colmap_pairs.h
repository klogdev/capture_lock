#include <Eigen/Core>
#include <vector>

using Matrix3x4d = Eigen::Matrix<double, 3, 4>;

std::string Trim(const std::string& str);

void LoadColmapPairs(const std::string& filename,
                     std::vector<std::vector<Eigen::Vector2d>>& points2D,
                     std::vector<std::vector<Eigen::Vector3d>>& points3D,
                     std::vector<Matrix3x4d>& composed_extrinsic);

void printData(const std::vector<std::vector<Eigen::Vector2d>>& points2D,
               const std::vector<std::vector<Eigen::Vector3d>>& points3D,
               const std::vector<Matrix3x4d>& composed_extrinsic);