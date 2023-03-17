#include "bundle_adjuster.h"
#include "base/image.h"
#include "base/point3d.h"

bool GlobalBundleAdjuster(const colmap::BundleAdjustmentOptions& ba_options
                        std::unordered_map<int,colmap::Image>& global_image_map,
                        std::unordered_map<int,colmap::Point3D>& global_3d_map);

