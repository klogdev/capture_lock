#include <Eigen/Core>
#include "file_reader/colmap_pairs.h"
using Matrix3x4d = Eigen::Matrix<double, 3, 4>;


int main() {
    std::string processed_colmap = "/tmp2/processed_pair.txt";

    std::vector<std::vector<Eigen::Vector2d>> points2D;
    std::vector<std::vector<Eigen::Vector3d>> points3D;
    std::vector<Matrix3x4d> composed_extrinsic;

    LoadColmapPairs(processed_colmap,
                    points2D, points3D, composed_extrinsic);
    
    printData(points2D, points3D, composed_extrinsic);

    return 0;
}