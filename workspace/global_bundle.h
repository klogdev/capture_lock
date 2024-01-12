#include "base/image.h"
#include "base/camera.h"
#include "base/point3d.h"

#include "optim/bundle_adjustment.h"

#include "file_reader/data_types.h"

bool GlobalBundleAdjuster(const colmap::BundleAdjustmentOptions& ba_options,
                          colmap::Camera& Camera,
                          std::unordered_map<int,colmap::Image>& global_image_map,
                          std::unordered_map<int, colmap::Point3D>& global_3d_map,
                          std::vector<int>& image_for_opt,
                          const std::vector<int>& const_pose,
                          Dataset data);

void GetSlideWindow(const int window_size, const int curr_idx,
                    std::vector<int>& curr_window, std::vector<int>& const_pose);

