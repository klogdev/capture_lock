#include <Eigen/Core>
#include <vector>
#include <string>

using Matrix3x4d = Eigen::Matrix<double, 3, 4>;

/**
 * @brief load usegeo pairs, the files are preprocessed via a separate
 * python script; the patterns of the files are as follows:
 * 
 * pose_i: qx, qy, qz, qw, tx, ty, tz (i.e. Hamiltonian convention)
 * u, v, x, y, z
 * .....
 * 
 * @param filename 
 * @param points2D 
 * @param points3D 
 * @param composed_extrinsic 
 */
void LoadUseGeo(const std::string& filename,
    std::vector<std::vector<Eigen::Vector2d>>& points2D,
    std::vector<std::vector<Eigen::Vector3d>>& points3D,
    std::vector<Matrix3x4d>& composed_extrinsic);
