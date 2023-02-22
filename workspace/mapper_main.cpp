#include <string>
#include <Eigen/Core>
#include "base/reconstruction.h"
#include "base/image.h"
#include "base/camera.h"
#include "base/triangulation.h"

#include "feature/image_sift.h"
#include "feature/sift.h"
#include "file_stream.h"
#include "incremental_construct.h"
#include "init_first_pair.h"

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
    colmap::Camera camera = read_text.Cameras().at(1);//need to change focal, due to downsampling

    std::vector<std::string> image_stream = FilePathStream(image_path);

    //start create map by init list of hashmaps
    std::unordered_map<int,colmap::Image> global_image_map;
    std::unordered_map<int,std::vector<sift::Keypoint>> global_keypts_map;
    std::unordered_map<int,Eigen::Vector3d> global_3d_map;

    //triangulate first two 
    InitFirstPair(image_stream[0], image_stream[1], camera,
                global_image_map,global_keypts_map,global_3d_map);
    //increment remaining frames
    for (int i = 2; i < image_stream.size(); i++){
        IncrementOneImage(image_stream[i], i, global_image_map[i-1],camera,
                        global_image_map,global_keypts_map,global_3d_map);
    }

    std::cout << global_3d_map.size() << std::endl;
    return 0;
}