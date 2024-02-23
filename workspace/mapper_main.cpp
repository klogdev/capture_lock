#include <string>
#include <Eigen/Core>
#include <map>

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

#include "incremental_construct.h"
#include "init_first_pair.h"
#include "bundle_adjuster.h"
#include "global_bundle.h"
#include "util_/pose_save.h"
#include "util_/kitti_pose.h"
#include "file_reader/file_stream.h"
#include "file_reader/file_options.h"
#include "file_reader/data_types.h"
#include "file_reader/kitti_odo_helper.h"
#include "file_reader/colmap_pose.h"

#include "util_/gt_map.h"

void DebugTracks(colmap::Track track){
    std::vector<colmap::TrackElement>& curr_vec = track.Elements();

    for (const auto elem: curr_vec){
        std::cout << elem.image_id << std::endl;
    }
}

int main(int argc, char** argv) {
    if (argc < 2)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << " need specify the dataset type" 
        << std::endl;
    }
    std::string datasetName = argv[1];

    Dataset dataset = getDatasetFromName(datasetName);

    // 1. instantiate file options based on file type, 
    // @file_reader/file_option.h
    FileOptions files_to_run;
    InstantiateFiles(files_to_run, dataset);

    // 2. factory pattern to load calibration info
    std::unique_ptr<CalibFileReader> file_reader = CalibFileReader::CalibFileCreate(dataset);
    colmap::Camera camera = file_reader->GetIntrinsicMat(files_to_run.calib_path,
                                                         files_to_run.seq_num,
                                                         files_to_run.downsample,
                                                         files_to_run.width,
                                                         files_to_run.height); 

    // read colmap's south building images of the first segment
    // or kitti's image with specified seq num
    std::vector<std::string> image_stream;
    if (dataset == Colmap) {
        COLMAPStream(image_stream, files_to_run.image_path, 
                        files_to_run.image_start, files_to_run.image_end);
    }
    else if (dataset == Kitti) {
        KITTIStream(image_stream, files_to_run.image_path,
                    files_to_run.image_start, files_to_run.image_end);
    }
    else if (dataset == KittiToColmap) {
        KITTIStream(image_stream, files_to_run.image_path,
                    files_to_run.image_start, files_to_run.image_end);
    }
    std::cout << "debug current data's image path is: " << 
    files_to_run.image_path << std::endl;

    // start create global maps by init list of hashmaps
    std::unordered_map<int,colmap::Image> global_image_map;
    std::unordered_map<int,std::vector<sift::Keypoint>> global_keypts_map;
    std::unordered_map<int,colmap::Point3D> global_3d_map;

    // init ba options, use soft l1 to be robust with outliers
    // this options will be reused for different local BA runs
    // principal_axis will not be refined by default in the ba_options
    colmap::BundleAdjustmentOptions ba_options;
    ba_options.solver_options.num_threads = 1;
    ba_options.loss_function_type = colmap::BundleAdjustmentOptions::LossFunctionType::SOFT_L1;
    ba_options.refine_focal_length = false;
    ba_options.refine_extra_params = false;

    // read g.t. poses, this request the user specifies the sequence num.
    // for colmap or colmap mapped kitti, we use "fake" seq num
    std::vector<std::vector<double>> extrinsic_gts;

    if(dataset == Kitti)
        ExtrinsicFromKitti(files_to_run.pose_path, files_to_run.seq_num,
                        extrinsic_gts);
    else if(dataset == Colmap || dataset == KittiToColmap)
        ExtrinsicFromColmap(files_to_run.pose_path, extrinsic_gts,
                            files_to_run.colmap_opt.image_name_start, 
                            files_to_run.colmap_opt.image_name_end);

    // specify the g.t. poses needed for initial and incremental processes
    // and init the g.t. poses map
    std::vector<int> gt_poses_num = {0, 1, 2};
    std::map<int, std::pair<Eigen::Vector4d,Eigen::Vector3d>> gt_map;
    if(dataset == Kitti) {
        CreateKittiGTMap(gt_map, gt_poses_num, extrinsic_gts);
    }
    else if(dataset == Colmap || dataset == KittiToColmap) {
        CreateColmapGTMap(gt_map, gt_poses_num, extrinsic_gts);
    }

    std::cout << "check g.t. poses: " << std::endl;
    std::cout << gt_map[0].first << std::endl;
    std::cout << gt_map[0].second << std::endl;
    
    // triangulate the first pair
    InitFirstPair(image_stream[0], image_stream[1], camera,
                  global_image_map, global_keypts_map, global_3d_map, 
                  (int)files_to_run.width*files_to_run.downsample,
                  (int)files_to_run.height*files_to_run.downsample,
                  gt_map[0].first, gt_map[0].second, 
                  gt_map[1].first, gt_map[1].second); // hard coded init pair

    std::vector<int> init_image_opt = {0, 1};
    std::vector<int> init_const_pose = {0, 1};

    // run ba for the first pair
    bool init_ba = GlobalBundleAdjuster(ba_options, camera, global_image_map,
                                        global_3d_map, init_image_opt, 
                                        init_const_pose, dataset);  
    // DEBUGGING: check the pose after init ba
    std::cout << "BA result for the first pair is: " << init_ba << std::endl;
    colmap::Image frame_1 = global_image_map[1];
    std::cout << "frame 1 pose after init BA is: " << std::endl;
    std::cout << frame_1.Qvec() << std::endl;
    std::cout << frame_1.Tvec() << std::endl;


    // manually init the sliding window size for local ba
    int window_size = 5;

    // increment remaining frames
    for (int i = 2; i < image_stream.size(); i++){
        IncrementOneImage(image_stream[i], i, i-1, camera,
                          global_image_map, global_keypts_map, global_3d_map,
                          gt_map, 
                          (int)files_to_run.width*files_to_run.downsample,
                          (int)files_to_run.height*files_to_run.downsample);
        std::cout << "num of 3d points after process image " << i << " is: " 
                  << global_3d_map.size() << std::endl;

        std::vector<int> curr_opt_window;
        std::vector<int> curr_const_pose;
        GetSlideWindow(window_size, i, curr_opt_window, curr_const_pose);
        // run ba for the local window
        bool local_ba = GlobalBundleAdjuster(ba_options, camera, global_image_map,
                                            global_3d_map, curr_opt_window, 
                                            curr_const_pose, dataset);  
        // DEBUGGING: check the pose after local ba
        std::cout << "BA result for the local ba " << i << " is: " << local_ba << std::endl;
        colmap::Image frame_i = global_image_map[curr_const_pose[0]];
        std::cout << "frame " << curr_const_pose[0] << " pose after local BA is: " << std::endl;
        std::cout << frame_i.Qvec() << std::endl;
        std::cout << frame_i.Tvec() << std::endl;
    }

    // run global ba, the fixed pose is still the 0'th frame
    std::vector<int> global_image_opt;
    for(const auto& [img_id, value]: global_image_map){
        global_image_opt.push_back(img_id);
    }

    std::vector<int> global_const_pose = {0};
    bool run_ba = GlobalBundleAdjuster(ba_options, camera, global_image_map,
                                       global_3d_map, global_image_opt, 
                                       global_const_pose, dataset);    

    std::cout << "result of global BA is: " << run_ba << std::endl;

    // save the poses
    std::string output = files_to_run.output;
    SavePoseToTxt(output, global_image_map);

    return 0;
}