#include "base/image.h"
#include "base/camera.h"
#include "base/point3d.h"
#include "optim/bundle_adjustment.h"

bool GlobalBundleAdjuster(const colmap::BundleAdjustmentOptions& ba_options,
                        colmap::Camera& Camera,
                        std::unordered_map<int,colmap::Image>& global_image_map,
                        std::unordered_map<int,colmap::Point3D>& global_3d_map);

