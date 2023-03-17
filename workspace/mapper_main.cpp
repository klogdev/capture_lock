#include <string>
#include <Eigen/Core>
#include "base/reconstruction.h"
#include "base/image.h"
#include "base/camera.h"
#include "base/point3d.h"
#include "base/triangulation.h"
#include "base/camera_models.h"
#include "optim/bundle_adjustment.h"

#include "feature/image_sift.h"
#include "feature/sift.h"
#include "file_stream.h"
#include "incremental_construct.h"
#include "init_first_pair.h"
#include "bundle_adjuster.h"

const double downscale_x = 768.0/3072;
const double downscale_y = 576.0/2304;
const double downscale_avg = (downscale_x + downscale_y)/2;

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

    std::vector<std::string> image_stream = FilePathStream(image_path);
    //start create map by init list of hashmaps
    std::unordered_map<int,colmap::Image> global_image_map;
    std::unordered_map<int,std::vector<sift::Keypoint>> global_keypts_map;
    std::unordered_map<int,colmap::Point3D> global_3d_map;

    //triangulate first two 
    InitFirstPair(image_stream[0], image_stream[1], camera,
                global_image_map,global_keypts_map,global_3d_map);

    //increment remaining frames
    for (int i = 2; i < image_stream.size(); i++){
        IncrementOneImage(image_stream[i], i, global_image_map[i-1],camera,
                        global_image_map,global_keypts_map,global_3d_map);
    }

    colmap::BundleAdjustmentOptions ba_options;
    bool run_ba = GlobalBundleAdjuster(ba_options, global_image_map, global_3d_map);    

    std::cout << "result of BA is: " << run_ba << std::endl;
    return 0;
}