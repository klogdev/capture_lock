#include <string>
#include <Eigen/Core>
#include "base/reconstruction.h"
#include "base/image.h"
#include "base/camera.h"

#include "base/projection.h"
#include "base/triangulation.h"
#include "estimators/pose.h"

#include "feature/sift.h"
#include "feature/image_sift.h"

#include "file_stream.h"
#include "init_first_pair.h"
#include "incremental_construct.h"

int main(int argc, char** argv){
    if (argc < 3) {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need sparse and images file path" << std::endl;
    }

    std::string sparse_path = argv[1];
    std::string image_path = argv[2];

    colmap::Reconstruction read_text = colmap::Reconstruction();
    read_text.ReadText(sparse_path);
    //assume we only have one camera
    colmap::Camera camera = read_text.Camera(1);
    
    camera.SetModelId(colmap::SimpleRadialCameraModel::model_id);
    std::vector<std::string> image_stream; 
    COLMAPStream(image_stream, image_path, 141, 150);

    // start create global maps by init list of hashmaps
    std::unordered_map<int,colmap::Image> global_image_map;
    std::unordered_map<int,std::vector<sift::Keypoint>> global_keypts_map;
    std::unordered_map<int,colmap::Point3D> global_3d_map;

    //triangulate first two 
    InitFirstPair(image_stream[0], image_stream[1], camera,
                global_image_map, global_keypts_map,global_3d_map, 3072, 2304);

    return 0;
}