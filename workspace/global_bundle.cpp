#include "global_bundle.h"
#include "bundle_adjuster.h"
#include "optim/bundle_adjustment.h"

bool GlobalBundleAdjuster(const colmap::BundleAdjustmentOptions& ba_options,
                          colmap::Camera& camera,
                          std::unordered_map<int,colmap::Image>& global_image_map,
                          std::unordered_map<int,colmap::Point3D>& global_3d_map){
    colmap::BundleAdjustmentConfig ba_config;
    for (const auto& [image_id, value] : global_image_map){
        std::cout << "Global Bundle added images: " << image_id << std::endl;
        ba_config.AddImage(image_id);
    }

    for (const auto& [point3d_id, value]: global_3d_map){
        ba_config.AddVariablePoint(point3d_id);
    }

    //set the first frame as constant, 0 indexed
    ba_config.SetConstantPose(0);

    //debug constant pose
    colmap::Image first_image = global_image_map[0];

    std::cout << "debug first quat: " << first_image.Qvec() << std::endl;
    std::cout << "debug first trans: " << first_image.Tvec() << std::endl;

  // Run bundle adjustment. SetUp is called inside the Solver
  BundleAdjust_ bundle_adjuster(ba_options, ba_config);
  if (!bundle_adjuster.Solver(camera, global_image_map, global_3d_map)) {
    return false;
  }

 return true;
}