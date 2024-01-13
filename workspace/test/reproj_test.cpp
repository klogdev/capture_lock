#include <string>
#include <Eigen/Core>
#include "base/reconstruction.h"
#include "base/image.h"
#include "base/camera.h"
#include "base/pose.h"
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
    // assume we only have one camera
    colmap::Camera camera = read_text.Camera(1);
    
    camera.SetModelId(colmap::SimpleRadialCameraModel::model_id);
    camera.Rescale(0.25);

    std::cout << "test cali matrix: " << std::endl;
    std::cout << camera.CalibrationMatrix() << std::endl;
    std::vector<std::string> image_stream; 
    COLMAPStream(image_stream, image_path, 141, 145);

    // start create global maps by init list of hashmaps
    std::unordered_map<int,colmap::Image> global_image_map;
    std::unordered_map<int,std::vector<sift::Keypoint>> global_keypts_map;
    std::unordered_map<int,colmap::Point3D> global_3d_map;

    // specify the g.t. poses for the first pair
    Eigen::Vector4d qvec_141 = Eigen::Vector4d(0.868254, 0.0224726, 0.474455, -0.143257);
    Eigen::Vector3d tvec_141 = Eigen::Vector3d(-0.679221, 1.00351, 3.65061);
    Eigen::Vector4d qvec_142 = Eigen::Vector4d(0.864347, 0.0331977, 0.477339, -0.154756);
    Eigen::Vector3d tvec_142 = Eigen::Vector3d(-0.756852, 0.980926, 3.58659);

    Eigen::Vector3d hard_3d = Eigen::Vector3d(2.87898, -0.441039, -0.516722);
    Eigen::Vector2d hard_2d = Eigen::Vector2d(465, 281);

    Eigen::Vector3d proj_point3D =
      colmap::QuaternionRotatePoint(qvec_141, hard_3d) + tvec_141;
    std::cout << "check depth of hard 3d: " << proj_point3D.z() << std::endl;
    double repro_hard = colmap::CalculateSquaredReprojectionError(hard_2d,
                                                                  hard_3d,
                                                                  qvec_142,
                                                                  tvec_142,
                                                                  camera);
    std::cout << "check the reprojection error of hard coded point: " << std::endl;
    std::cout << repro_hard << std::endl;
    // triangulate first pair
    InitFirstPair(image_stream[0], image_stream[1], camera,
                global_image_map, global_keypts_map,global_3d_map, 768, 576,
                qvec_141, tvec_141, qvec_142, tvec_142);

    return 0;
}