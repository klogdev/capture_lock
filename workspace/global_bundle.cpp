#include "global_bundle.h"
#include "bundle_adjuster.h"

#include "optim/bundle_adjustment.h"

#include "base/point2d.h"
#include "base/image.h"

#include "file_reader/data_types.h"

bool GlobalBundleAdjuster(const colmap::BundleAdjustmentOptions& ba_options,
                          colmap::Camera& camera,
                          std::unordered_map<int,colmap::Image>& global_image_map,
                          std::unordered_map<int, colmap::Point3D>& global_3d_map,
                          std::vector<int>& image_to_opt,
                          const std::vector<int>& const_pose,
                          Dataset data){
    colmap::BundleAdjustmentConfig ba_config;

    for (const int image_id : image_to_opt){
        // add image id via input indices
        ba_config.AddImage(image_id);

        colmap::Image curr_image = global_image_map[image_id];

        // add 3d points id via added images
        for(const colmap::Point2D& pts_2d: curr_image.Points2D()){
          if(!pts_2d.HasPoint3D())
            continue;
          colmap::point3D_t id_3d = pts_2d.Point3DId();
          ba_config.AddVariablePoint(id_3d);
        }
    }

    // set the constant poses based on input, 0 indexed
    std::cout << "constant poses id from global bundle with image from " 
    << image_to_opt[0] << ": " << std::endl;
    for(const int id: const_pose){
      std::cout << id << std::endl;
      ba_config.SetConstantPose(id);
    }

    // debug constant pose
    colmap::Image first_image = global_image_map[const_pose[0]];

    std::cout << "debug " << const_pose[0] << "'s quat before BA: " << std::endl;
    std::cout << first_image.Qvec() << std::endl;
    std::cout << "debug " << const_pose[0] << "'s trans before BA: " << std::endl;
    std::cout << first_image.Tvec() << std::endl;

  // Run bundle adjustment. SetUp is called inside the Solver
  // the bundle_adjuster_ will initialized by input image based on the
  // indices offered by ba_config
  BundleAdjust_ bundle_adjuster(ba_options, ba_config, data);
  std::cout << "DEBUG: no segfault before local BA " << const_pose[0] << std::endl;
  if (!bundle_adjuster.Solver(camera, global_image_map, global_3d_map)) {
    return false;
  }

 return true;
}

void GetSlideWindow(const int window_size, const int curr_idx,
                    std::vector<int>& curr_window, std::vector<int>& const_pose){
  int left_end = curr_idx + 1 - window_size;
  if(left_end < 0)
    left_end = 0;

  const_pose.push_back(left_end);

  for(int i = 0; i < std::min(window_size, curr_idx+1); i++){
    curr_window.push_back(left_end+i);
  }

  std::cout << "curr window from GetWindow is: " << std::endl;
  for(int w: curr_window){
      std::cout << w << std::endl;
  }
  std::cout << "curr constant poses from GetWindow is: " << std::endl;
  for(int p: const_pose){
      std::cout << p << std::endl;
  }
}