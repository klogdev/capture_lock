#include <Eigen/Core>
#include <memory>

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <sstream>
#include <random>
#include <cmath>
#include <limits>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "base/pose.h"
#include "util/random.h"

#include "estimate/lhm.h"
#include "util_/math_helper.h"
#include "pnp/pnp_test_data.h"
#include "pnp/pnp_helper.h"
#include "file_reader/tum_rgbd.h"

std::unique_ptr<DataGenerator> 
DataGenerator::createDataGenerator(const GeneratorType type) {
    switch (type) {
        case GeneratorType::EPnPdZ:
            return std::make_unique<BoxCornerEPnPTestDataDz>(BoxCornerEPnPTestDataDz());
        case GeneratorType::EPnPdY:
            return std::make_unique<BoxCornerEPnPTestDataDy>(BoxCornerEPnPTestDataDy());
        case GeneratorType::RandomNoise:
            return std::make_unique<BoxRandomEPnPTestDataNoise>(BoxRandomEPnPTestDataNoise());
        case GeneratorType::NumPts:
            return std::make_unique<BoxRandomTestNumPts>(BoxRandomTestNumPts());
        case GeneratorType::Outlier:
            return std::make_unique<BoxRandomOutliers>(BoxRandomOutliers());
        case GeneratorType::PlanarChk:
            return std::make_unique<BoxCornerPlanarSanity>(BoxCornerPlanarSanity());
        case GeneratorType::TUM:
            return std::make_unique<TumRgbd>(TumRgbd());
        case GeneratorType::EPnPSimNoise:
            return std::make_unique<EPnPSimulatorNoise>(EPnPSimulatorNoise());
        case GeneratorType::EPnPSimNum:
            return std::make_unique<EPnPSimulatorNumPts>(EPnPSimulatorNumPts());
        case GeneratorType::EPnPSimOutlier:
            return std::make_unique<EPnPSimulatorOutliers>(EPnPSimulatorOutliers());
        // Handle unsupported types
        default:
            return nullptr;
    }
}

// set default values for static members
double BoxCornerEPnPTestDataDz::sigma = 0.0003; // 70dB used by LHM
void BoxCornerEPnPTestDataDz::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                       std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                       std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    // set the default intrinsic matrix and Box corners
    Eigen::Matrix3d k; // = Eigen::Matrix3d::Identity();
    GetIntrinsic(k);
    std::vector<Eigen::Vector3d> camera_space_points;
    EPnPBoxCorner(camera_space_points);

    int num_samples = 500;
    
    double d_min = 0;
    double d_max = 150;

    for(double d = d_min; d <= d_max; d += 5) {
        for(int i = 0; i < num_samples; i++) {
            std::vector<Eigen::Vector3d> curr_points3d;
            Eigen::Vector3d curr_trans = Eigen::Vector3d(0, 0, d); // in camera space
            // shift to the desired dist inside camera
            std::vector<Eigen::Vector3d> shifted;
            CameraSpaceShift(camera_space_points, curr_trans, shifted);
            // got related noised 2D points
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(shifted, curr_points2d, k, BoxCornerEPnPTestDataDz::sigma);
            // shifted the CoM to overlap with camera's origin
            std::vector<Eigen::Vector3d> world_pts;
            CameraSpaceShift(shifted, Eigen::Vector3d(0,0,-6-d), world_pts);
            // generate world pts with random rotation
            Eigen::Matrix3d curr_rot;
            EPnPRandomRot(curr_rot);
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot.transpose()),
                                                Eigen::Vector3d(0,0,0));
            // generate scene points from non-noised camera points
            for (size_t i = 0; i < world_pts.size(); i++) {
                Eigen::Vector3d point3D_world = world_pts[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            points2D.push_back(curr_points2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            // converting [R|t] to [R^T|-R^T*t]
            curr_gt.block<3, 3>(0, 0) = curr_rot;
            curr_gt.col(3) = Eigen::Vector3d(0,0,6+d);
            composed_extrinsic.push_back(curr_gt);
        }
    }
}

// set default values for static members
double BoxCornerEPnPTestDataDy::sigma = 0.0003;
void BoxCornerEPnPTestDataDy::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                       std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                       std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    // set the default intrinsic matrix and Box corners
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    std::vector<Eigen::Vector3d> camera_space_points;
    EPnPBoxCorner(camera_space_points);

    Eigen::Matrix3d k_inv = k.inverse();

    int num_samples = 500;

    double d_min = 5;
    double d_max = 500;

    for(double d = d_min; d <= d_max; d += 10) {
        for(int i = 0; i < num_samples; i++) {
            Eigen::Matrix3d curr_rot;
            EPnPRandomRot(curr_rot);
            std::vector<Eigen::Vector3d> curr_points3d;
            Eigen::Vector3d curr_trans = Eigen::Vector3d(5, d, 200);
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot),
                                                curr_trans);
            // generate scene points
            for (size_t i = 0; i < camera_space_points.size(); i++) {
                Eigen::Vector3d point3D_world = camera_space_points[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from a single camera points set
           std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(camera_space_points, curr_points2d, k, BoxCornerEPnPTestDataDy::sigma);

            points2D.push_back(curr_points2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot.transpose(); 
            curr_gt.col(3) = -curr_rot.transpose()*curr_trans; 
            composed_extrinsic.push_back(curr_gt);
        }
    }
}

// set default values for static members
double BoxRandomEPnPTestDataNoise::sigma_s = 1.0;
double BoxRandomEPnPTestDataNoise::sigma_e = 15.0;
void BoxRandomEPnPTestDataNoise::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                          std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                          std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    int num_pts = 6;
    Eigen::Matrix3d k_inv = k.inverse();

    int num_sample = 500;

    for(double i = BoxRandomEPnPTestDataNoise::sigma_s; i <= BoxRandomEPnPTestDataNoise::sigma_e; i += 1.0) {
        for(int j = 0; j < num_sample; j++) {
            // generate one set of camera space points
            std::vector<Eigen::Vector3d> curr_camera_space;
            EPnPInsideRand(curr_camera_space, num_pts);

            // generate noised 2d points from camera space points
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, i);

            Eigen::Matrix3d curr_rot;
            Eigen::Vector3d curr_trans;
            EPnPRandomRot(curr_rot);
            EPnPRandomTrans(curr_trans);

            std::vector<Eigen::Vector3d> curr_points3d;
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot),
                                                curr_trans);
            // generate scene points
            for (size_t i = 0; i < curr_camera_space.size(); i++) {
                Eigen::Vector3d point3D_world = curr_camera_space[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from a single camera points set
            points2D.push_back(curr_points2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot.transpose(); 
            curr_gt.col(3) = -curr_rot.transpose()*curr_trans; 
            composed_extrinsic.push_back(curr_gt);
        }
    }
}

int BoxRandomTestNumPts::min_pts = 5;
int BoxRandomTestNumPts::max_pts = 50;
double BoxRandomTestNumPts::sigma = 5.0;
void BoxRandomTestNumPts::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                   std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                   std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    Eigen::Matrix3d k_inv = k.inverse();

    int num_sample = 500;
    for(int i = BoxRandomTestNumPts::min_pts; i <= BoxRandomTestNumPts::max_pts; i++) {
        for(int j = 0; j < num_sample; j++) {
            // generate one set of camera space points
            std::vector<Eigen::Vector3d> curr_camera_space;
            EPnPInsideRand(curr_camera_space, i);

            // generate noised 2d points from camera space points
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, BoxRandomTestNumPts::sigma);

            Eigen::Matrix3d curr_rot;
            Eigen::Vector3d curr_trans;
            EPnPRandomRot(curr_rot);
            EPnPRandomTrans(curr_trans);

            std::vector<Eigen::Vector3d> curr_points3d;
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot),
                                                curr_trans);
            // generate scene points
            for (size_t i = 0; i < curr_camera_space.size(); i++) {
                Eigen::Vector3d point3D_world = curr_camera_space[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from a single camera points set
            points2D.push_back(curr_points2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot.transpose(); 
            curr_gt.col(3) = -curr_rot.transpose()*curr_trans; 
            composed_extrinsic.push_back(curr_gt);
        }
    }
}

double BoxRandomOutliers::percent_s = 0.05; // 5%
double BoxRandomOutliers::percent_e = 0.25;
void BoxRandomOutliers::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                 std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                 std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    Eigen::Matrix3d k_inv = k.inverse();

    int num_samples = 10;
    for(double i = BoxRandomOutliers::percent_s; i <= BoxRandomOutliers::percent_e; i += 0.05) {
        for(int j = 0; j < num_samples; j++) {
            // generate one set of camera space points with fixed # 20
            std::vector<Eigen::Vector3d> curr_camera_space;
            EPnPInsideRand(curr_camera_space, 20);

            // generate noised 2d points from camera space points
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, 5.0);
            AddOutlier2D(curr_points2d, i, 640, 480);

            Eigen::Matrix3d curr_rot;
            Eigen::Vector3d curr_trans;
            EPnPRandomRot(curr_rot);
            EPnPRandomTrans(curr_trans);

            std::vector<Eigen::Vector3d> curr_points3d;
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot),
                                                curr_trans);
            // generate scene points
            for (size_t i = 0; i < curr_camera_space.size(); i++) {
                Eigen::Vector3d point3D_world = curr_camera_space[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from a single camera points set
            points2D.push_back(curr_points2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot.transpose(); 
            curr_gt.col(3) = -curr_rot.transpose()*curr_trans; 
            composed_extrinsic.push_back(curr_gt);
        }
    }
}

std::string TumRgbd::depth_parent = "/tmp3/tum_rgbd/depth/";
std::string TumRgbd::align_pose = "/tmp3/Pose_PnP/aligned_poses.txt";
void TumRgbd::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                       std::vector<std::vector<Eigen::Vector3d>>& points3D,
                       std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    std::vector<boost::filesystem::path> depth_files;
    GetSortedFiles(TumRgbd::depth_parent, depth_files);

    std::vector<std::string> depth_strings;
    for (const auto& path : depth_files) {
        depth_strings.push_back(path.string());  // Convert path to string and add to the list
    }

    TUMIntrinsic tum_para = TUMIntrinsic();
    ProcessAllPairs(depth_strings, TumRgbd::align_pose, tum_para, points2D, points3D, composed_extrinsic);
}

double EPnPSimulatorNoise::sigma_s = 1.0;
double EPnPSimulatorNoise::sigma_e = 15.0;
void EPnPSimulatorNoise::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                             std::vector<std::vector<Eigen::Vector3d>>& points3D,
                             std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    int num_pts = 6;
    Eigen::Matrix3d k_inv = k.inverse();

    int num_sample = 500;
    for(double i = EPnPSimulatorNoise::sigma_s; i <= EPnPSimulatorNoise::sigma_e; i += 1.0) {
        for(int j = 0; j < num_sample; j++) {
            std::vector<Eigen::Vector3d> curr_camera_space;
            EPnPInsideRand(curr_camera_space, 6);
            // obtain the projected points as usual
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, i);
            // shift the cloud to (0,0,0) via regarding CoM of cloud
            // as the world's origin
            Eigen::Vector3d curr_trans;
            CalculateCoM(curr_camera_space, curr_trans);
            std::vector<Eigen::Vector3d> shifted_camera;
            CameraSpaceShift(curr_camera_space, -curr_trans, shifted_camera);

            Eigen::Matrix3d curr_rot;
            EPnPRandomRot(curr_rot);

            std::vector<Eigen::Vector3d> curr_points3d;
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot.transpose()),
                                                Eigen::Vector3d(0,0,0));
            // generate scene points
            for (size_t i = 0; i < shifted_camera.size(); i++) {
                Eigen::Vector3d point3D_world = shifted_camera[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from a single camera points set
            points2D.push_back(curr_points2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot; 
            curr_gt.col(3) = curr_trans; 
            composed_extrinsic.push_back(curr_gt);            
        }
    }
}

int EPnPSimulatorNumPts::max_pts = 20;
int EPnPSimulatorNumPts::min_pts = 5;
double EPnPSimulatorNumPts::sigma = 5.0;
void EPnPSimulatorNumPts::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                   std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                   std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    Eigen::Matrix3d k_inv = k.inverse();

    int num_sample = 500;
    for(int i = EPnPSimulatorNumPts::min_pts; i <= EPnPSimulatorNumPts::max_pts; i++) {
        for(int j = 0; j < num_sample; j++) {
            std::vector<Eigen::Vector3d> curr_camera_space;
            EPnPInsideRand(curr_camera_space, i);
            // obtain the projected points as usual
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, EPnPSimulatorNumPts::sigma);
            // shift the cloud to (0,0,0) via regarding CoM of cloud
            // as the world's origin
            Eigen::Vector3d curr_trans;
            CalculateCoM(curr_camera_space, curr_trans);
            std::vector<Eigen::Vector3d> shifted_camera;
            CameraSpaceShift(curr_camera_space, -curr_trans, shifted_camera);

            Eigen::Matrix3d curr_rot;
            EPnPRandomRot(curr_rot);

            std::vector<Eigen::Vector3d> curr_points3d;
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot.transpose()),
                                                Eigen::Vector3d(0,0,0));
            // generate scene points
            for (size_t i = 0; i < shifted_camera.size(); i++) {
                Eigen::Vector3d point3D_world = shifted_camera[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from a single camera points set
            points2D.push_back(curr_points2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot; 
            curr_gt.col(3) = curr_trans; 
            composed_extrinsic.push_back(curr_gt);            
        }
    }
}

int EPnPSimulatorOutliers::max_pts = 20;
int EPnPSimulatorOutliers::min_pts = 5;
double EPnPSimulatorOutliers::sigma = 5.0;
void EPnPSimulatorOutliers::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                   std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                   std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    Eigen::Matrix3d k_inv = k.inverse();

    int num_sample = 500;
    for(int i = EPnPSimulatorOutliers::min_pts; i <= EPnPSimulatorOutliers::max_pts; i++) {
        for(int j = 0; j < num_sample; j++) {
            std::vector<Eigen::Vector3d> curr_camera_space;
            EPnPInsideRand(curr_camera_space, i);
            // obtain the projected points as usual
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, EPnPSimulatorOutliers::sigma);
            AddOutlier2D(curr_points2d, i, 640, 480);
            // shift the cloud to (0,0,0) via regarding CoM of cloud
            // as the world's origin
            Eigen::Vector3d curr_trans;
            CalculateCoM(curr_camera_space, curr_trans);
            std::vector<Eigen::Vector3d> shifted_camera;
            CameraSpaceShift(curr_camera_space, -curr_trans, shifted_camera);

            Eigen::Matrix3d curr_rot;
            EPnPRandomRot(curr_rot);

            std::vector<Eigen::Vector3d> curr_points3d;
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot.transpose()),
                                                Eigen::Vector3d(0,0,0));
            // generate scene points
            for (size_t i = 0; i < shifted_camera.size(); i++) {
                Eigen::Vector3d point3D_world = shifted_camera[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from a single camera points set
            points2D.push_back(curr_points2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot; 
            curr_gt.col(3) = curr_trans; 
            composed_extrinsic.push_back(curr_gt);            
        }
    }
}

double BoxCornerPlanarSanity::sigma_s = 0.05;
double BoxCornerPlanarSanity::sigma_e = 0.65;
std::string BoxCornerPlanarSanity::option = "planar";
void BoxCornerPlanarSanity::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                     std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                     std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    Eigen::Matrix3d k_inv = k.inverse();

    int num_sample = 500;
    for(double i = BoxCornerPlanarSanity::sigma_s; i <= BoxCornerPlanarSanity::sigma_e; i += 0.1) {
        for(int j = 0; j < num_sample; j++) {
            // generate one set of camera space points
            std::vector<Eigen::Vector3d> curr_camera_space;
            if(BoxCornerPlanarSanity::option == "box") {
                EPnPBoxCorner(curr_camera_space);
            }
            else {
                EPnPPlanar(curr_camera_space);
            }
            
            Perturbation3D(curr_camera_space, i);

            // generate noised 2d points from camera space points
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, 0.5);

            Eigen::Matrix3d curr_rot;
            Eigen::Vector3d curr_trans(5, 5, 15);
            EPnPRandomRot(curr_rot);

            std::vector<Eigen::Vector3d> curr_points3d;
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot),
                                                curr_trans);
            // generate scene points
            for (size_t i = 0; i < curr_camera_space.size(); i++) {
                Eigen::Vector3d point3D_world = curr_camera_space[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from a single camera points set
            points2D.push_back(curr_points2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot.transpose(); 
            curr_gt.col(3) = -curr_rot.transpose()*curr_trans; 
            composed_extrinsic.push_back(curr_gt);
        }
    }
}