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

void DebugTracks(colmap::Track track){
    std::vector<colmap::TrackElement>& curr_vec = track.Elements();

    for (const auto elem: curr_vec){
        std::cout << elem.image_id << std::endl;
    }
}

int main(int argc, char** argv){
    if (argc < 2)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << " need specify the dataset type" 
        << std::endl;
    }
    std::string datasetName = argv[1];

    Dataset dataset = getDatasetFromName(datasetName);

    FileOptions files_to_run;
    InstantiateFiles(files_to_run, dataset);

    std::vector<Eigen::Matrix3x4d> test_init;

    std::unique_ptr<CalibFileReader> file_reader = CalibFileReader::CalibFileCreate(dataset);

    colmap::Camera camera = file_reader->GetIntrinsicMat(files_to_run.calib_path,
                                                         files_to_run.seq_num,
                                                         files_to_run.downsample,
                                                         files_to_run.width,
                                                         files_to_run.height); 

    // read colmap's south building images of the first segment
    std::vector<std::string> image_stream;
    if (dataset == Colmap) {
        COLMAPStream(image_stream, files_to_run.image_path, 
                        files_to_run.start, files_to_run.end);
    }
    else if (dataset == Kitti) {
        std::cout << "enter the Kitti stream option" << std::endl;
        KITTIStream(image_stream, files_to_run.image_path,
                    files_to_run.start, files_to_run.end);
    }
    std::cout << "debug current data's image path is: " << 
    files_to_run.image_path << std::endl;


    // start create global maps by init list of hashmaps
    std::unordered_map<int,colmap::Image> global_image_map;
    std::unordered_map<int,std::vector<sift::Keypoint>> global_keypts_map;
    std::unordered_map<int,colmap::Point3D> global_3d_map;

    // init ba options, use soft l1 to be robust with outliers
    // this options will be reused for different local BA runs
    // principal_axis will not be refined by default in ba_options
    colmap::BundleAdjustmentOptions ba_options;
    ba_options.solver_options.num_threads = 1;
    ba_options.loss_function_type = colmap::BundleAdjustmentOptions::LossFunctionType::SOFT_L1;
    ba_options.refine_focal_length = false;
    ba_options.refine_extra_params = false;

    // specify the g.t. poses for the first pair
    Eigen::Vector4d qvec_141 = Eigen::Vector4d(0.868254, 0.0224726, 0.474455, -0.143257);
    Eigen::Vector3d tvec_141 = Eigen::Vector3d(-0.679221, 1.00351, 3.65061);
    Eigen::Vector4d qvec_167 = Eigen::Vector4d(0.0986641, 0.00817242, 0.950918, -0.293179);
    Eigen::Vector3d tvec_167 = Eigen::Vector3d(-0.127192, 0.869954, 2.9168);
    // override the pose of second frame with colmap's g.t.
    Eigen::Vector4d qvec_142 = Eigen::Vector4d(0.864347, 0.0331977, 0.477339, -0.154756);
    Eigen::Vector3d tvec_142 = Eigen::Vector3d(-0.756852, 0.980926, 3.58659);
    Eigen::Vector4d qvec_168 = Eigen::Vector4d(0.0986538, 0.00897312, 0.955091, -0.279263);
    Eigen::Vector3d tvec_168 = Eigen::Vector3d(-0.352987, 0.768079, 2.86639);
    // read kitti's g.t. poses, this request the user specifies the sequence num
    // std::vector<std::vector<double>> extrinsic_kitti;
    // ExtrinsicFromKitti(files_to_run.pose_path, files_to_run.seq_num,
    //                 extrinsic_kitti);
    // const Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> kitti_frame0(extrinsic_kitti[0].data());
    // const Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> kitti_frame1(extrinsic_kitti[1].data());

    // init the g.t. poses for the first pair of Kitti
    Eigen::Vector4d qvec_kitti0;
    Eigen::Vector3d tvec_kitti0;
    Eigen::Vector4d qvec_kitti1;
    Eigen::Vector3d tvec_kitti1;

    // QuatTransFromExtrinsic(qvec_kitti0, tvec_kitti0, kitti_frame0);
    // QuatTransFromExtrinsic(qvec_kitti1, tvec_kitti1, kitti_frame1);
    // triangulate the first pair
    InitFirstPair(image_stream[0], image_stream[1], camera,
                  global_image_map, global_keypts_map, global_3d_map, 
                  768, // (int)files_to_run.width*files_to_run.downsample,
                  576, // (int)files_to_run.height*files_to_run.downsample,
                  qvec_141, tvec_141, qvec_142, tvec_142); // hard coded init pair

    std::vector<int> init_image_opt = {0,1};
    std::vector<int> init_const_pose = {0};

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
    int window_size = 3;

    // increment remaining frames
    for (int i = 2; i < image_stream.size(); i++){
        IncrementOneImage(image_stream[i], i, i-1, camera,
                        global_image_map, global_keypts_map, global_3d_map, 768, 576);
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
        // debugging number of tracked features after all frames
        // for(const auto& point: global_3d_map){
        //     colmap::Point3D curr_3d = point.second;
        //     int curr_track_len = curr_3d.Track().Length();
        //     if(curr_3d.Track().Length() > 5){
        //         std::cout << "the 3D point " << point.first << " has tracked images: " << std::endl;
        //         DebugTracks(curr_3d.Track());
        //     }
        // }
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