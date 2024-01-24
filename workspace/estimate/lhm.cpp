#include <Eigen/Core>
#include "estimate/lhm.h"

bool LHMEstimator::ComputeLHMPose(const std::vector<Eigen::Vector2d>& points2D,
                                  const std::vector<Eigen::Vector3d>& points3D,
                                  Eigen::Matrix3x4d* proj_matrix) {
    int n_points = points2D.size();
    std::vector<Eigen::Matrix3d> V;
}