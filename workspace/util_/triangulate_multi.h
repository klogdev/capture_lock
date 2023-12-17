#include <Eigen/Core>
#include "base/triangulation.h"

Eigen::Vector3d TriangulateMultiViewPoint(
    const std::vector<Eigen::Matrix3x4d>& cams_from_world,
    const std::vector<Eigen::Vector2d>& points);