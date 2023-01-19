#include "base/camera.h"
#include "base/image.h"
#include "base/point2d.h"
#include "base/point3d.h"
#include "base/reconstruction.h"
#include "util/types.h"
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace {

using ::colmap::Camera;
using ::colmap::Image;
using ::colmap::Point2D;
using ::colmap::Point3D;
using ::colmap::Reconstruction;

long int UniformLongRandom(long int NumImages) {
  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 gen(rd());  // seed the generator
  std::uniform_int_distribution<> distr(0, NumImages);  // define the range

  int idx = distr(gen);  // generate numbers
  return idx;
}

//calculate reprojection error
double ProjDiff(Eigen::Vector2d Orig2D, Eigen::Vector3d Proj2D){
    return std::abs(pow(Orig2D[0] - Proj2D[0]/Proj2D[2], 2) + pow(Orig2D[1] - Proj2D[1]/Proj2D[2], 2));
}

}  // namespace

//check exe file of COLMAP, read image files from argument
int main(int argc, char** argv){
    if (argc < 4)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "<image> <camera> <point3d>" << std::endl;
    }

    //get files name from command line
    std::string image_file = argv[1];
    std::string camera_file = argv[2];
    std::string point3d_file = argv[3];
    //read files and instantiate class for each
    ReadData read_image = ReadData(image_file);
    ReadData read_camera = ReadData(camera_file);
    ReadData read_point3d = ReadData(point3d_file);
    long int n = read_image.Images().size();  //number of images by access member data

    long int idx1 = UniformLongRandom();
    long int idx2 = UniformLongRandom();

    // image instances and corresponding data already set in image class
    // images_ is a map of images
    //  TODO: read_image.ReadImagesText(filename); //filename initialized in
    //  constructor
    Image Image1 = read_image.Images().at(idx1);  // check image_t == image_id??
    Image Image2 = read_image.Images().at(idx2);

#if 0
    //set boolean when read 3d idx from image.txt
    while (! read_image.Images()[idx1].HasPoint3D()){
        idx1 = UniformLongRandom();
    }

    while (! read_image.Images()[idx2].HasPoint3D() || idx2 == idx1){
        idx2 = UniformLongRandom();
    }
#endif
    // create camera for image1&2, read camera paras
    //  TODO: read_camera.ReadCamerasText(filename);
    int camera_id1 = Image1.CameraId();
    int camera_id2 = Image2.CameraId();
    Camera camera1 = read_camera.Cameras().at(camera_id1);
    Camera camera2 = read_camera.Cameras().at(camera_id2);
    std::vector<double> cam_para = camera1.Params();  // check def of params??
    std::vector<size_t> cam_focal = camera1.FocalLengthIdxs();

    Eigen::Matrix3d calibration = camera1.CalibrationMatrix();

    //read 3d, find consistent in point3D_, which is a member variable of ReadData
    //Test 3d point's reprojection for camera 1
    camera_t Cam1Point2D_id = UniformLongRandom();
    Point2D Cam1Point = Image1.Points2D()[Cam1Point2D_id];
    //call correspond 3d point from its attr
    //point2d need add point3d accessors
    camera_t Cam1Point3D_id = Cam1Point.Point3DId();
    Point3D Cam1Point3D = read_point3d.Points3D().at(Cam1Point3D_id);

    Eigen::Vector3d Cam1Point3D_Vec = Cam1Point3D.XYZ();
    Eigen::Matrix3x4d Image1ProjMat = Image1.ProjectionMatrix();

    //generate a 3d points in homo space for projection
    Eigen::Vector4d Cam1Point3D_Homo = Eigen::Vector4d::Identity();
    Cam1Point3D_Homo.topRows(3) = Cam1Point3D_Vec; 

    Eigen::Vector3d Projected2D = calibration*Image1ProjMat*Cam1Point3D_Homo;

    double diff = ProjDiff(Cam1Point.XY(), Projected2D);
    std::cout << "difference of one projection"  << diff << std::endl;

    return 0;
}
