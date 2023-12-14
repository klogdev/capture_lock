#include "base/camera.h"
#include "base/image.h"
#include "base/point2d.h"
#include "base/point3d.h"
#include "base/reconstruction.h"
#include "util/types.h"

#include <Eigen/Core>
#include <random>
#include <string>
#include <vector>

//calculate reprojection error
double ReprojErr(Eigen::Vector2d& point_2d, Eigen::Vector3d& point_3d,
                colmap::Camera camera, Eigen:Matrix3x4d& Proj_matrix);