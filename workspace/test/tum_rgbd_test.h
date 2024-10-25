#ifndef TEST_TUM_RGBD_TEST_H_
#define TEST_TUM_RGBD_TEST_H_

#include <Eigen/Core>

#include "base/camera.h"
#include "base/camera_models.h"
#include "base/image.h"
#include "base/point3d.h"

void CheckTUMResidual(colmap::Image& curr_image, colmap::Camera& camera,
                      std::unordered_map<int, colmap::Point3D>& global_3d_map);

#endif // TEST_TUM_RGBD_TEST_H_