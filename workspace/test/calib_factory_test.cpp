#include <string>
#include "kitti_calib.h"
#include "colmap_calib.h"

#include "base/camera.h"

//test the factory pattern of calibration file reader
int main(int argc, char** argv){
    if (argc < 3)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need two calibration file paths" << std::endl;
    }

    std::string colmap_path = argv[1];
    std::string kitti_path = argv[2];

    CalibFileReader colmap_reader = CalibFileReader::create("colmap");
    CalibFileReader kitti_reader = CalibFileReader::create("kitti");

    colmap::Camera colmap_camera = colmap_reader.GetIntrinsicMat(colmap_path,"",1.0);
    colmap::Camera kitti_camera = kitti_reader.GetIntrinsicMat(kitti_path,"00",1.0);

    Eigen::Matrix3d colmap_calib = colmap_camera.CalibrationMatrix();
    Eigen::Matrix3d kitti_calib = kitti_camera.CalibrationMatrix();

    std::cout << "colmap's cali: " << std::endl;
    std::cout << colmap_calib << std::endl;
    std::cout << "kitti's cali: " << std::endl;
    std::cout << kitti_calib << std::endl;

    return 0;
}