#include <iostream>
#include <string>

#include "calib_base.h"
#include "base/camera_model.h"

struct FileOptions{
    std::string calib_path;
    std::string image_path;
    std::string seq_num;
    std::string output;
    int camera_model;
    float downsample;
};

void InstantiateFiles(FileOptions& files, std::string dataset){
    if(dataset == "colmap"){
        files.calib_path = "/tmp2/sparse";
        files.image_path = "/tmp2/image";
        files.camera_model = colmap::SimpleRadialCameraModel::model_id;
        files.downsample = 0.25;
        file.seq_num = '0';
    }
    else if(dataset == "kitti"){
        files.calib_path = "/tmp3/KITTI_Odometry/gray/";
        files.image_path = "/tmp3/KITTI_Odometry/gray/image_0";
        files.camera_model = colmap::SimpleRadialCameraModel::model_id;
        files.downsample = 1.0;
        files.seq_num = "00";
    }
}
