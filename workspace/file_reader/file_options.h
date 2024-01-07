#include <iostream>
#include <string>

#include "calib_loader/calib_base.h"
#include "base/camera_models.h"
#include "data_types.h"

struct FileOptions{
    std::string calib_path;
    std::string image_path;
    std::string pose_path;
    std::string seq_num;
    std::string output;
    int camera_model;
    float downsample;
    int width;
    int height;
    int start;
    int end;
};

void InstantiateFiles(FileOptions& files, Dataset dataset){
    if(dataset == Dataset::Colmap){
        files.calib_path = "/tmp2/sparse/";
        files.image_path = "/tmp2/images/";
        files.camera_model = colmap::SimpleRadialCameraModel::model_id;
        files.downsample = 0.25;
        files.seq_num = "0";
        files.width = 3072;
        files.height = 2304;
        files.start = 141;
        files.end = 150;
        files.output = "/tmp2/optim_poses.txt";
    }
    else if(dataset == Dataset::Kitti){
        files.calib_path = "/tmp3/KITTI_Odometry/gray/";
        files.image_path = "/tmp3/KITTI_Odometry/gray/image_0";
        files.pose_path = "/tmp3/KITTI_Odometry/poses/";
        files.camera_model = colmap::SimpleRadialCameraModel::model_id;
        files.downsample = 1.0;
        files.seq_num = "00";
        files.width = 1241;
        files.height = 376;
        files.output = "/tmp3/KITTI_Odometry/kitti_poses.txt";
    }
}
