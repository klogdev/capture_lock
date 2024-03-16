#include <iostream>
#include <string>

#include "calib_loader/calib_base.h"
#include "base/camera_models.h"
#include "data_types.h"

/**
 * @brief here start/end are image file name's string indices
 * in Colmap file format 
*/
struct ColmapFileOptions {
    int image_name_start;
    int image_name_end;
};

struct FileOptions {
    std::string calib_path;
    std::string image_path;
    std::string pose_path;
    std::string seq_num;
    std::string output;
    int camera_model;
    float downsample;
    int width;
    int height;
    int image_start;
    int image_end;
    ColmapFileOptions colmap_opt;
};

/**
 * @brief instantiate file options by a simple if-else
*/
void InstantiateFiles(FileOptions& files, Dataset dataset) {
    if(dataset == Dataset::Colmap) {
        files.calib_path = "/tmp2/sparse/";
        files.pose_path = "/tmp2/sparse/images.txt";
        files.image_path = "/tmp2/images/";
        files.camera_model = colmap::SimpleRadialCameraModel::model_id;
        files.downsample = 0.25;
        files.seq_num = "0";
        files.width = 3072;
        files.height = 2304;
        files.image_start = 141;
        files.image_end = 146;
        files.output = "/tmp2/optim_poses.txt";
        files.colmap_opt.image_name_start = 5;
        files.colmap_opt.image_name_end = 8;
    }
    else if(dataset == Dataset::Kitti) {
        files.calib_path = "/tmp3/KITTI_Odometry/gray/";
        files.seq_num = "03";
        files.image_path = files.calib_path + files.seq_num + "/image_0";
        files.pose_path = "/tmp3/KITTI_Odometry/poses/"; // extrinsic reader will add seq num inside
        files.camera_model = colmap::SimplePinholeCameraModel::model_id;
        files.downsample = 1.0;
        files.width = 1242;
        files.height = 375;
        files.image_start = 0;
        files.image_end = 15;
        files.output = "/tmp3/KITTI_Odometry/kitti_poses_15_seq3_rel_epnp.txt";
    }
    else if(dataset == Dataset::KittiToColmap) {
        files.seq_num = "00";
        files.calib_path = "/tmp3/KITTI_Odometry/" + files.seq_num + "_txt/";
        files.image_path = "/tmp3/KITTI_Odometry/gray/" + files.seq_num + "/image_0";
        files.pose_path = "/tmp3/KITTI_Odometry/" + files.seq_num + "_txt/images.txt";
        files.camera_model = colmap::SimplePinholeCameraModel::model_id;
        files.downsample = 1.0;
        files.width = 1241;
        files.height = 376;
        files.image_start = 0;
        files.image_end = 20;
        files.output = "/tmp3/KITTI_Odometry/" + files.seq_num + "_txt/kitti_poses_20_gt6.txt";
        files.colmap_opt.image_name_start = 0;
        files.colmap_opt.image_name_end = 6;
    }
}
