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
#include "file_reader/colmap_pairs.h"
#include "file_reader/orb_loaded.h"
#include "file_reader/use_geo.h"

std::unique_ptr<DataGenerator> 
DataGenerator::createDataGenerator(const GeneratorType type) {
    switch (type) {
        case GeneratorType::EPnPdZ:
            return std::make_unique<BoxCornerEPnPDataDz>(BoxCornerEPnPDataDz());
        case GeneratorType::EPnPdY:
            return std::make_unique<BoxCornerEPnPTestDataDy>(BoxCornerEPnPTestDataDy());
        case GeneratorType::Outlier:
            return std::make_unique<OutliersPercentage>(OutliersPercentage());
        case GeneratorType::PlanarChk:
            return std::make_unique<PlanarCase>(PlanarCase());
        case GeneratorType::PlanarPtb:
            return std::make_unique<PlanarPerturb>(PlanarPerturb());
        case GeneratorType::BoxPtb:
            return std::make_unique<BoxPerturb>(BoxPerturb());
        case GeneratorType::TUM:
            return std::make_unique<TumRgbd>(TumRgbd());
        case GeneratorType::COLMAP:
            return std::make_unique<ColmapPair>(ColmapPair());
        case GeneratorType::ORB:
            return std::make_unique<OrbGenerate>(OrbGenerate());
        case GeneratorType::EPnPSimNoise:
            return std::make_unique<EPnPSimulatorNoise>(EPnPSimulatorNoise());
        case GeneratorType::EPnPSimNum:
            return std::make_unique<EPnPSimulatorNumPts>(EPnPSimulatorNumPts());
        case GeneratorType::POSITCube:
            return std::make_unique<PositCube>(PositCube());
        case GeneratorType::UseGeo:
            return std::make_unique<UseGeo>(UseGeo());
        // Handle unsupported types
        default:
            return nullptr;
    }
}

void PositCube::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D,
                        std::vector<std::vector<Eigen::Vector3d>>& points3D,
                        std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    // Define test data
    const double focal_length = 760.0;
    
    // Single set of 3D points of the cube
    std::vector<Eigen::Vector3d> cube_3d = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(10, 0, 0),
        Eigen::Vector3d(10, 10, 0),
        Eigen::Vector3d(0, 10, 0),
        Eigen::Vector3d(0, 0, 10),
        Eigen::Vector3d(10, 0, 10),
        Eigen::Vector3d(10, 10, 10),
        Eigen::Vector3d(0, 10, 10)
    };

    // Single set of 2D image points (normalized by dividing by focal length)
    std::vector<Eigen::Vector2d> cube_2d = {
        Eigen::Vector2d(0.0 / focal_length, 0.0 / focal_length),
        Eigen::Vector2d(80.0 / focal_length, -93.0 / focal_length),
        Eigen::Vector2d(245.0 / focal_length, -77.0 / focal_length),
        Eigen::Vector2d(185.0 / focal_length, 32.0 / focal_length),
        Eigen::Vector2d(32.0 / focal_length, 135.0 / focal_length),
        Eigen::Vector2d(99.0 / focal_length, 35.0 / focal_length),
        Eigen::Vector2d(247.0 / focal_length, 62.0 / focal_length),
        Eigen::Vector2d(195.0 / focal_length, 179.0 / focal_length)
    };

    // Ground truth rotation matrix
    Eigen::Matrix3d R;
    R << 0.38074924, 0.91705712, 0.11847469,
         -0.58652078, 0.17730644, 0.79028843,
         0.70373331, -0.37038959, 0.60538235;

    // Ground truth translation vector
    Eigen::Vector3d t(0.0, 0.0, 41.91170542);

    // Compose the projection matrix [R|t]
    Eigen::Matrix3x4d proj_matrix;
    proj_matrix.block<3,3>(0,0) = R;
    proj_matrix.col(3) = t;

    // Clear and add single test case
    points2D.clear();
    points3D.clear();
    composed_extrinsic.clear();
    
    points2D.push_back(cube_2d);
    points3D.push_back(cube_3d);
    composed_extrinsic.push_back(proj_matrix);
}

// set default values for static members
double BoxCornerEPnPDataDz::sigma = 5.0; 
void BoxCornerEPnPDataDz::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                   std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                   std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    // set the default intrinsic matrix and Box corners
    Eigen::Matrix3d k; // = Eigen::Matrix3d::Identity();
    GetIntrinsic(k);
    std::vector<Eigen::Vector3d> camera_space_points;
    EPnPBoxCorner(camera_space_points);

    int num_samples = 500;
    
    double d_min = 0;
    double d_max = 100;

    for(double d = d_min; d <= d_max; d += 5) {
        for(int i = 0; i < num_samples; i++) {
            std::vector<Eigen::Vector3d> curr_points3d;
            Eigen::Vector3d curr_trans = Eigen::Vector3d(0, 0, d); // in camera space
            // shift to the desired dist inside camera
            std::vector<Eigen::Vector3d> shifted;
            CameraSpaceShift(camera_space_points, curr_trans, shifted);
            // got related noised 2D points
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(shifted, curr_points2d, k, BoxCornerEPnPDataDz::sigma);
            // shifted the CoM to overlap with camera's origin
            // -6 because the box's com in camera space is at (0,0,6)
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

double OutliersPercentage::percent_s = 0.05; // 5%
double OutliersPercentage::percent_e = 0.35; // 35%
void OutliersPercentage::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                 std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                 std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    Eigen::Matrix3d k_inv = k.inverse();

    int num_samples = 500;
    for(double i = OutliersPercentage::percent_s; i <= OutliersPercentage::percent_e; i += 0.05) {
        for(int j = 0; j < num_samples; j++) {
            // generate one set of camera space points with fixed # 20
            std::vector<Eigen::Vector3d> curr_camera_space;
            EPnPInsideRand(curr_camera_space, 30);

            // generate noised 2d points from camera space points
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, 2.0);
            AddOutlier2D(curr_points2d, i, 640, 480, k_inv);

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

std::string TumRgbd::curr_data = "rgbd_dataset_freiburg2_rpy";
std::string TumRgbd::image_parent = "/tmp3/dataset/tum_rgbd/" + TumRgbd::curr_data + "/rgb/";
std::string TumRgbd::depth_parent = "/tmp3/dataset/tum_rgbd/" + TumRgbd::curr_data + "/depth/";
std::string TumRgbd::align_pose = "/tmp3/dataset/tum_rgbd/" + TumRgbd::curr_data + "/aligned_poses.txt";
void TumRgbd::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                       std::vector<std::vector<Eigen::Vector3d>>& points3D,
                       std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    std::vector<boost::filesystem::path> depth_files;
    std::vector<boost::filesystem::path> image_files;
    DepthRGBMap(TumRgbd::image_parent, TumRgbd::depth_parent, image_files, depth_files);

    std::vector<std::string> depth_strings;
    for (const auto& path : depth_files) {
        depth_strings.push_back(path.string());  // Convert path to string and add to the list
    }

    std::vector<std::string> image_strings;
    for (const auto& path : image_files) {
        image_strings.push_back(path.string());  // Convert path to string and add to the list
    }

    TUMIntrinsic tum_para = TUMIntrinsic();
    ProcessAllPairs(image_strings, depth_strings, TumRgbd::align_pose, tum_para, points2D, points3D, composed_extrinsic);
}

std::string OrbGenerate::processed_orb = "/tmp3/dataset/tum_rgbd/rgbd_dataset_freiburg1_teddy/AllPairs.txt";
bool OrbGenerate::add_outliers = false;
double OrbGenerate::outliers_percent = 0.3; // 30%
int OrbGenerate::num_repeat = 20;
int OrbGenerate::num_each_frame = 10;
int OrbGenerate::seq_num = 1;
void OrbGenerate::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                           std::vector<std::vector<Eigen::Vector3d>>& points3D,
                           std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    std::vector<std::vector<Eigen::Vector2d>> points2D_raw;
    std::vector<std::vector<Eigen::Vector3d>> points3D_raw;
    std::vector<Eigen::Matrix3x4d> composed_extrinsic_raw;
    LoadOrbLoaded(OrbGenerate::processed_orb, points2D_raw,
                  points3D_raw, composed_extrinsic_raw, OrbGenerate::seq_num);

    for (int i = 0; i < num_repeat; i++) {
        for (int j = 0; j < points2D_raw.size(); j++) {
            std::vector<Eigen::Vector2d> curr_points2D;
            std::vector<Eigen::Vector3d> curr_points3D;
            // generate one set of 2d points and 3d points
            std::unordered_set<int> idx_set;
            int cnt = 0;
            while (cnt < num_each_frame) {
                int rand = GenerateRandomInt(0, points2D_raw[j].size() - 1);
                if (idx_set.count(rand) != 0) continue;
                idx_set.insert(rand);
                curr_points2D.push_back(points2D_raw[j][rand]);
                curr_points3D.push_back(points3D_raw[j][rand]);
                cnt++;
            }
            points2D.push_back(curr_points2D);
            points3D.push_back(curr_points3D);
            composed_extrinsic.push_back(composed_extrinsic_raw[j]);
        }
    }
    if (OrbGenerate::add_outliers) {
        TUMIntrinsic tum_para = TUMIntrinsic();
        Eigen::Matrix3d k;
        k << tum_para.fx, 0, tum_para.cx,
             0, tum_para.fy, tum_para.cy,
             0, 0, 1;
        Eigen::Matrix3d k_inv = k.inverse();
        for(int i = 0; i < points2D.size(); i++) {
            AddOutlier2D(points2D[i], OrbGenerate::outliers_percent, 640, 480, k_inv);
        }
        
    }
}

std::string ColmapPair::processed_colmap = "/tmp3/dataset/co3d/broccoli/63_46_colmap/processed_pair.txt";
double ColmapPair::radius_small = 0.04;
double ColmapPair::radius_large = 0.20;
int ColmapPair::num_repeat = 20;
int ColmapPair::num_each_frame = 15;
void ColmapPair::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                          std::vector<std::vector<Eigen::Vector3d>>& points3D,
                          std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    std::vector<std::vector<Eigen::Vector2d>> points2D_raw;
    std::vector<std::vector<Eigen::Vector3d>> points3D_raw;
    std::vector<Eigen::Matrix3x4d> composed_extrinsic_raw;
    LoadColmapPairs(ColmapPair::processed_colmap, points2D_raw,
                    points3D_raw, composed_extrinsic_raw);
    std::vector<int> valid_frame_indices;
    double smallest_radius = ColmapPair::radius_small;
    
    for (int j = 0; j < points2D_raw.size(); j++) {
        std::vector<Eigen::Vector2d> filtered_points2D;
        std::vector<Eigen::Vector3d> filtered_points3D;
        FilterByNormalizedRadius(points2D_raw[j], points3D_raw[j], filtered_points2D, filtered_points3D, smallest_radius);
        
        if (filtered_points2D.size() >= ColmapPair::num_each_frame) {
            valid_frame_indices.push_back(j);
        }
    }

    std::cout << "Valid frame indices size: " << valid_frame_indices.size() << std::endl;
                    
    for(double i = ColmapPair::radius_small; i <= ColmapPair::radius_large; i += 0.04) {
        for(int k = 0; k < ColmapPair::num_repeat; k++) {
            for(int j = 0; j < valid_frame_indices.size(); j++) {
                int idx = valid_frame_indices[j];
                std::vector<Eigen::Vector2d> filtered_points2D;
                std::vector<Eigen::Vector3d> filtered_points3D;
                FilterByNormalizedRadius(points2D_raw[idx], points3D_raw[idx], filtered_points2D, filtered_points3D, i);
                if (filtered_points2D.size() < ColmapPair::num_each_frame) {
                    std::cout << "Filtered points size " << filtered_points2D.size() << 
                                        " is not enough to generate " << ColmapPair::num_each_frame << 
                                        " points with radius " << i << std::endl;
                    continue;
                }

                // generate one set of 2d points and 3d points
                std::vector<Eigen::Vector2d> curr_points2D;
                std::vector<Eigen::Vector3d> curr_points3D;
                std::unordered_set<int> idx_set;
                int cnt = 0;
                while (cnt < ColmapPair::num_each_frame) {
                    int rand = GenerateRandomInt(0, filtered_points2D.size() - 1);
                    if (idx_set.count(rand) != 0) continue;
                    idx_set.insert(rand);
                    curr_points2D.push_back(filtered_points2D[rand]);
                    curr_points3D.push_back(filtered_points3D[rand]);
                    cnt++;
                }
                points2D.push_back(curr_points2D);
                points3D.push_back(curr_points3D);
                composed_extrinsic.push_back(composed_extrinsic_raw[idx]);
            }
        }
    }
}

std::string UseGeo::processed_usegeo = "/tmp3/dataset/usegeo/processed_usegeo.txt";
int UseGeo::num_repeat = 20;
int UseGeo::num_each_frame = 50;
void UseGeo::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                          std::vector<std::vector<Eigen::Vector3d>>& points3D,
                          std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    std::vector<std::vector<Eigen::Vector2d>> points2D_raw;
    std::vector<std::vector<Eigen::Vector3d>> points3D_raw;
    std::vector<Eigen::Matrix3x4d> composed_extrinsic_raw;
    LoadUseGeo(UseGeo::processed_usegeo, points2D_raw, points3D_raw, composed_extrinsic_raw);

    for(int i = 0; i < num_repeat; i++) {
        for(int j = 0; j < points2D_raw.size(); j++) {
            std::vector<Eigen::Vector2d> curr_points2D;
            std::vector<Eigen::Vector3d> curr_points3D;
            // generate one set of 2d points and 3d points
            std::unordered_set<int> idx_set;
            int cnt = 0;
            while (cnt < num_each_frame) {
                int rand = GenerateRandomInt(0, points2D_raw[j].size() - 1);
                if (idx_set.count(rand) != 0) continue;
                idx_set.insert(rand);
                curr_points2D.push_back(points2D_raw[j][rand]);
                curr_points3D.push_back(points3D_raw[j][rand]);
                cnt++;
            }
            points2D.push_back(curr_points2D);
            points3D.push_back(curr_points3D);
            composed_extrinsic.push_back(composed_extrinsic_raw[j]);
        }
    }
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
            EPnPInsideRand(curr_camera_space, 8);
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


double PlanarCase::vertices_s = 1;
double PlanarCase::vertices_e = 10;
bool PlanarCase::tilt = false;
void PlanarCase::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                          std::vector<std::vector<Eigen::Vector3d>>& points3D,
                          std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    Eigen::Matrix3d k_inv = k.inverse();

    int num_sample = 500;
    for(int i = PlanarCase::vertices_s; i <= PlanarCase::vertices_e; i++) {
        for(int j = 0; j < num_sample; j++) {
            std::vector<Eigen::Vector3d> curr_camera_space;
            EPnPPlanarRand(curr_camera_space, 50 - i); // total 50 points, remove i out of plane points
            std::vector<Eigen::Vector3d> curr_out_of_plane;
            EPnPInsideRand(curr_out_of_plane, i);
            curr_camera_space.insert(curr_camera_space.end(), curr_out_of_plane.begin(), curr_out_of_plane.end());

            if(PlanarCase::tilt) PlanarTilt(curr_camera_space);
            // obtain the projected points as usual
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, 1);
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

double PlanarPerturb::sigma_s = 0.02;
double PlanarPerturb::sigma_e = 0.5;
bool PlanarPerturb::tilt = false;
void PlanarPerturb::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                          std::vector<std::vector<Eigen::Vector3d>>& points3D,
                          std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    Eigen::Matrix3d k_inv = k.inverse();

    int num_sample = 500;
    for(double i = PlanarPerturb::sigma_s; i <= PlanarPerturb::sigma_e; i += 0.02) {
        for(int j = 0; j < num_sample; j++) {
            std::vector<Eigen::Vector3d> curr_camera_space;
            EPnPPlanarRand(curr_camera_space, 8); // total 30 points

            if(PlanarPerturb::tilt) PlanarTilt(curr_camera_space);
            // perturb 3d points
            Perturbation3D(curr_camera_space, i);
            // obtain the projected points as usual
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, 1);
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

double BoxPerturb::sigma_s = 0.02;
double BoxPerturb::sigma_e = 0.5;
void BoxPerturb::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                          std::vector<std::vector<Eigen::Vector3d>>& points3D,
                          std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    Eigen::Matrix3d k_inv = k.inverse();

    int num_sample = 500;
    for(double i = BoxPerturb::sigma_s; i <= BoxPerturb::sigma_e; i += 0.02) {
        for(int j = 0; j < num_sample; j++) {
            std::vector<Eigen::Vector3d> curr_camera_space;
            EPnPBoxCorner(curr_camera_space); 

            // perturb 3d points
            Perturbation3D(curr_camera_space, i);
            // obtain the projected points as usual
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, 1);
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