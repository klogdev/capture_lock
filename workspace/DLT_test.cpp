#include <vector>
#include <string>
#include <random>
#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include "read_data.h"

long int random()
{
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(25, 63); // define the range

    
    int idx =  distr(gen) ; // generate numbers
    return idx;
}

//function to read input control points (required 6) and correspond image points
//output: 3x4 projection matrix
Eigen::Matrix3x4d DLT(vector<Eigen::Vector3d> ControlPoints, vector<Eigen::Vector2d> ImagePoints)
{
    if(ControlPoints.size() < 6)
        throw std::invalid_argument( "DLT requires at least 6 calibration points.");
    
    vector<vector<float>> LinearSystem;
    int nPoints = ControlPoints.size();
    for(int i = 0; i < nPoints; i++)
    {
        Point3D curr_3d = ControlPoints[i];
        Point2D curr_2d = ImagePoints[i];
        float x, y, z = curr_3d.X(), curr_3d.Y(), curr_3d.Z();
        float u, v = curr_2d.X(), curr_2d.Y();
        vector<float> curr_1{x, y, z, 1, 0, 0, 0, 0, -u*x, -u*y, -u*z, -u};
        vector<float> curr_2{0, 0, 0, 0, x, y, z, 1, -v*x, -v*y, -v*z, -v}; //does the negative sign matters?
        LinearSystem.push_back(curr_1);
        LinearSystem.push_back(curr_2);
    }

    //initialize solution vec and homogeneaous vec
    VectorXd b = Eigen::Map<VectorXd>(LinearSystem.size());
    VectorXd x = Eigen::Map<VectorXd>(LinearSystem.size());

    //convert 2d vector as coeff matrix, DLTmat defined in type.h
    int n_rows, n_cols = LinearSystem.size(), LinearSystem[0].size();
    DLTMatrix M = Eigen::Map<DLTMatrix>(LinearSystem.data(), n_rows, n_cols);
    
    //solve the linear system;(could also solved by SVD)
    x = M.colPivHouseholderQr().solve(b);

    Eigen::Matrix3x4d ProjMatrix = Eigen::Map<Matrix3x4d>(x.data(), 3, 4);;

    return ProjMatrix;
}

//get calibration mat from projection mat
Eigen::Matrix3d CalibrationMatFromQR(Eigen::Matrix3x4d P){
    
}

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
    int n = read_image.Images().size();  //number of images by access member data
    unordered_set<long> pickPoints;
    while (pickPoints.size() < 6){
        long currIdx = random();
        pickPoints.insert(currIdx);
    }
    
    //image instances and corresponding data already set in image class
    //images_ is a map of images
    read_image.ReadImagesText(filename); //filename initialized in constructor
    Image Image1 = read_image.Images()[idx1]; //check image_t == image_id??
    Image Image2 = read_image.Images()[idx2];

    //set boolean when read 3d idx from image.txt
    while (! read_image.Images()[idx1].HasPoint3D()){
        idx1 = random();
    }

    while (! read_image.Images()[idx2].HasPoint3D() || idx2 == idx1){
        idx2 = random();
    }

    //create camera for image1&2, read camera paras
    read_camera.ReadCamerasText(filename);
    int camera_id1 = Image1.CameraId();
    int camera_id2 = Image2.CameraId();
    Camera camera1 = read_camera.Cameras()[camera_id1];
    Camera camera2 = read_camera.Cameras()[camera_id2];
    std::vector<size_t> cam_para = camera1.Params();//check def of params??
    std::vector<size_t> cam_focal = camera1.FocalLengthIdxs();

    Eigen::Matrix3d calibration = camera1.CalibrationMatrix();

    //read 3d, find consistent in point3D_, which is a member variable of ReadData
    //Test 3d point's reprojection for camera 1
    uint32_t Cam1Point2D_id = random();
    Point2D Cam1Point = Image1.Points2D()[Cam1Point2D_id];
    //call correspond 3d point from its attr
    //point2d need add point3d accessors
    uint32_t Cam1Point3D_id = Cam1Point.Point3DId();
    Point3D Cam1Point3D = read_point3d.Points3D()[Cam1Point3D_id];

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
