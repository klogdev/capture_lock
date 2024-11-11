#ifndef FILE_READER_ORB_LOADED_H_
#define FILE_READER_ORB_LOADED_H_

#include <Eigen/Core>
#include <vector>

void LoadOrbLoaded(const std::string& filename,
                   std::vector<std::vector<Eigen::Vector2d>>& points2D,
                   std::vector<std::vector<Eigen::Vector3d>>& points3D,
                   std::vector<Eigen::Matrix3x4d>& composed_extrinsic);

#endif // FILE_READER_ORB_LOADED_H_