#include <string>
#include <Eigen/Core>
#include "base/reconstruction.h"
#include "base/image.h"
#include "base/camera.h"
#include "base/point3d.h"
#include "base/triangulation.h"
#include "base/camera_models.h"
#include "base/track.h"
#include "optim/bundle_adjustment.h"

#include "feature/image_sift.h"
#include "feature/sift.h"
#include "file_stream.h"
#include "incremental_construct.h"
#include "init_first_pair.h"
#include "bundle_adjuster.h"
#include "global_bundle.h"
#include "pose_save.h"

const double downscale_x = 768.0/3072; // dowsacale of the image
const double downscale_y = 576.0/2304;
const double downscale_avg = (downscale_x + downscale_y)/2;

void DebugTracks(colmap::Track track){
    std::vector<colmap::TrackElement>& curr_vec = track.Elements();

    for (const auto elem: curr_vec){
        std::cout << elem.image_id << std::endl;
    }
}

int main(int argc, char** argv){
    if (argc < 3)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "path to 3 .txt data files and image files" 
        << std::endl;
    }
    std::string sparse_path = argv[1];
    std::string image_path = argv[2];

    colmap::Reconstruction read_text = colmap::Reconstruction();
    read_text.ReadText(sparse_path);
    //assume we only have one camera
    colmap::Camera camera = read_text.Camera(1);//need to change focal, due to downsampling
    std::cout << "debug camera_model_info " << camera.ModelId() << " " 
              << camera.ModelName() << " " << camera.Width() << " " 
              << camera.Height() << " " << camera.ParamsToString() << std::endl;
    camera.SetModelId(colmap::SimpleRadialCameraModel::model_id);
    camera.Rescale(downscale_x);

    std::vector<std::vector<std::string>> image_stream = FilePathStream(image_path);
    //start create map by init list of hashmaps
    std::unordered_map<int,colmap::Image> global_image_map;
    std::unordered_map<int,std::vector<sift::Keypoint>> global_keypts_map;
    std::unordered_map<int,colmap::Point3D> global_3d_map;

    //triangulate first two 
    InitFirstPair(image_stream[0][0], image_stream[1][1], camera,
                global_image_map,global_keypts_map,global_3d_map,576,768);

    //increment remaining frames
    for (int i = 2; i < image_stream[0].size(); i++){
        IncrementOneImage(image_stream[0][i], i, global_image_map[i-1],camera,
                        global_image_map,global_keypts_map,global_3d_map,768,576);
        std::cout << "num of 3d points after process image " << i << " is: " 
                  << global_3d_map.size() << std::endl;

        //debugging number of tracked features after all frames
        for(const auto& point: global_3d_map){
            colmap::Point3D curr_3d = point.second;
            int curr_track_len = curr_3d.Track().Length();
            if(curr_3d.Track().Length() > 5){
                std::cout << "the 3D point " << point.first << " has tracked images: " << std::endl;
                DebugTracks(curr_3d.Track());
            }
        }
    }


    // init ba options, use soft l1 to be robust with outliers
    colmap::BundleAdjustmentOptions ba_options;
    ba_options.solver_options.num_threads = 1;
    ba_options.loss_function_type = colmap::BundleAdjustmentOptions::LossFunctionType::SOFT_L1;

    bool run_ba = GlobalBundleAdjuster(ba_options, camera, global_image_map, global_3d_map);    

    std::cout << "result of BA is: " << run_ba << std::endl;

    //save the poses
    std::string output = "/tmp2/optim_poses.txt";
    SavePoseToTxt(output, global_image_map);

    return 0;
}