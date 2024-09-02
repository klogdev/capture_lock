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

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "base/pose.h"
#include "util/random.h"

#include "estimate/lhm.h"
#include "util_/math_helper.h"
#include "test/pnp_test_data.h"

std::unique_ptr<DataGenerator> 
DataGenerator::createDataGenerator(const GeneratorType type) {
    switch (type) {
        case GeneratorType::CVLab:
            return std::make_unique<CVLabTestData>(CVLabTestData());
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
        // Handle unsupported types
        default:
            return nullptr;
    }
}

// set default values for static members
double BoxCornerEPnPTestDataDz::sigma = 0.0003;
void BoxCornerEPnPTestDataDz::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                       std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                       std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    // set the default intrinsic matrix and Box corners
    Eigen::Matrix3d k;
    GetIntrinsic(k);
    std::vector<Eigen::Vector3d> camera_space_points;
    EPnPBoxCorner(camera_space_points);

    Eigen::Matrix3d k_inv = k.inverse();

    int num_samples = 500;
    
    double d_min = 35;
    double d_max = 35;

    for(double d = d_min; d <= d_max; d += 10) {
        Eigen::Matrix3d curr_rot;
        EPnPRandomRot(curr_rot);
        for(int i = 0; i < num_samples; i++) {
            Eigen::Matrix3d curr_rot;
            EPnPRandomRot(curr_rot);
            std::vector<Eigen::Vector3d> curr_points3d;
            Eigen::Vector3d curr_trans = Eigen::Vector3d(5, 15, d);
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot),
                                                curr_trans);
            // generate scene points from non-noised camera points
            for (size_t i = 0; i < camera_space_points.size(); i++) {
                Eigen::Vector3d point3D_world = camera_space_points[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from an identical set camera points set
            std::vector<Eigen::Vector2d> curr_points2d;
            GenOneSetNoise2D(camera_space_points, curr_points2d, k, BoxCornerEPnPTestDataDz::sigma);

            points2D.push_back(curr_points2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            // converting [R|t] to [-R^T|-R^T*t]
            curr_gt.block<3, 3>(0, 0) = curr_rot.transpose(); 
            curr_gt.col(3) = -curr_rot.transpose()*curr_trans; 
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

    for(int i = BoxRandomEPnPTestDataNoise::sigma_s; i <= BoxRandomEPnPTestDataNoise::sigma_e; i++) {

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
            GenOneSetNoise2D(curr_camera_space, curr_points2d, k, 5.0);

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

void GetIntrinsic(Eigen::Matrix3d& k) {
    k << 800, 0, 320,
         0, 800, 240,
         0, 0, 1;
}

void GenOneSetNoise2D(std::vector<Eigen::Vector3d>& camera_space_points, 
                      std::vector<Eigen::Vector2d>& one_set_2d,
                      Eigen::Matrix3d& k, double sigma) {
    Eigen::Matrix3d k_inv = k.inverse();

    for(int i = 0; i < camera_space_points.size(); i++) {
        Eigen::Vector3d curr_camera_pt = camera_space_points[i];
        Eigen::Vector3d curr_projected = k*curr_camera_pt;
        double curr_pix_x = curr_projected.x()/curr_projected.z();
        double curr_pix_y = curr_projected.y()/curr_projected.z();
        Eigen::Vector3d noised_image_pt = 
                        Eigen::Vector3d(RandomGaussian(curr_pix_x, sigma),
                                        RandomGaussian(curr_pix_y, sigma),
                                        1);
        Eigen::Vector3d noised_camera_pt = k_inv*noised_image_pt;
        one_set_2d.push_back(Eigen::Vector2d(noised_camera_pt.x()/noised_camera_pt.z(),
                                    noised_camera_pt.y()/noised_camera_pt.z()));
    }
}

void Perturbation3D(std::vector<Eigen::Vector3d>& camera_space_points, double sigma) {
    // Create a random engine and a Gaussian distribution
    std::random_device rd;  // Seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::normal_distribution<> dis(0, sigma); // mean 0 and standard deviation sigma

    // Apply Gaussian noise to each point
    for (Eigen::Vector3d& p : camera_space_points) {
        p.x() += dis(gen);
        p.y() += dis(gen);
        p.z() += dis(gen);
    }
}

void AddOutlier2D(std::vector<Eigen::Vector2d>& points2D, double outlier_rate, 
                  const int image_x, const int image_y) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(0, image_x);
    std::uniform_real_distribution<> dis_y(0, image_y);
    
    int num_points = points2D.size();
    int num_outliers = static_cast<int>(num_points * outlier_rate);
    
    for (int i = 0; i < num_outliers; i++) {
        int index = rand() % num_points;
        points2D[index] = Eigen::Vector2d(dis_x(gen), dis_y(gen)); // Random outlier
    }
}

void CVLabTestData::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                             std::vector<std::vector<Eigen::Vector3d>>& points3D,
                             std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    std::string path_2d = file_path + "2d.txt";
    std::string path_3d = file_path + "3d.txt";
    std::string path_k = file_path + "k.txt";

    Eigen::Matrix3d K;
    SetIntrinsic(path_k, K); // load intrinsic to normalize 2d points
    Eigen::Matrix3d k_inv = K.inverse();

    std::vector<Eigen::Vector2d> one_set_2d; // 2d pixel points vector
    ReadCVLab2D(path_2d, one_set_2d);
    std::vector<Eigen::Vector2d> normalized_2d; // one set of normalized 2d points by applying k^-1

    for(const auto& p : one_set_2d) {
        Eigen::Vector3d homogeneousPoint(p[0], p[1], 1.0);
        Eigen::Vector3d homo_point = k_inv*homogeneousPoint;
        Eigen::Vector2d normalized(homo_point[0], homo_point[1]);
        normalized_2d.push_back(normalized);
    }
    points2D.push_back(normalized_2d);

    std::vector<Eigen::Vector3d> one_points3D;
    ReadCVLab3D(path_3d, one_points3D);
    points3D.push_back(one_points3D);

    Eigen::Matrix3x4d dummy_gt = Eigen::Matrix3x4d::Zero();
    composed_extrinsic.push_back(dummy_gt);
}

//////////////////////////////////////////
// Helper function for EPnP synthetic data
/////////////////////////////////////////
void EPnPBoxCorner(std::vector<Eigen::Vector3d>& camera_space_points) {

    // Define the ranges for x, y, and z
    double x_values[2] = {-2, 2};
    double y_values[2] = {-2, 2};
    double z_values[2] = {4, 8};

    // Generate the corners
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                camera_space_points.push_back(Eigen::Vector3d(x_values[i], y_values[j], z_values[k]));
            }
        }
    }
}

void EPnPPlanar(std::vector<Eigen::Vector3d>& camera_space_points) {
    // Define the ranges for x and y, and set a fixed z value
    double x_values[3] = {-2, 0, 2};  // Using three positions along x
    double y_values[3] = {-2, 0, 2};  // Using three positions along y
    double fixed_z_value = 6;  // Choose a mid-range value between 4 and 8

    // Generate points on the plane
    // Select 8 points including corners, midpoints of edges, and center point
    camera_space_points.push_back(Eigen::Vector3d(x_values[0], y_values[0], fixed_z_value)); // Corner
    camera_space_points.push_back(Eigen::Vector3d(x_values[0], y_values[1], fixed_z_value)); // Edge midpoint
    camera_space_points.push_back(Eigen::Vector3d(x_values[0], y_values[2], fixed_z_value)); // Corner
    camera_space_points.push_back(Eigen::Vector3d(x_values[1], y_values[0], fixed_z_value)); // Edge midpoint
    camera_space_points.push_back(Eigen::Vector3d(x_values[1], y_values[2], fixed_z_value)); // Edge midpoint
    camera_space_points.push_back(Eigen::Vector3d(x_values[2], y_values[0], fixed_z_value)); // Corner
    camera_space_points.push_back(Eigen::Vector3d(x_values[2], y_values[1], fixed_z_value)); // Edge midpoint
    camera_space_points.push_back(Eigen::Vector3d(x_values[2], y_values[2], fixed_z_value)); // Corner
}

void EPnPInsideRand(std::vector<Eigen::Vector3d>& camera_space_points,
                    int num_pts) {
    for(int i = 0; i < num_pts; i++) {
        double curr_x = RandomUniform(-2, 2);
        double curr_y = RandomUniform(-2, 2);
        double curr_z = RandomUniform(4, 8);
        camera_space_points.push_back(Eigen::Vector3d(curr_x, curr_y, curr_z));
    }
}

void EPnPRandomRot(Eigen::Matrix3d& rot) {
    double alpha = RandomUniform(0, 45);
    double beta = RandomUniform(0, 45);
    double gamma = RandomUniform(0, 45);

    alpha = alpha * M_PI / 180.0;
    beta = beta * M_PI / 180.0;
    gamma = gamma * M_PI / 180.0;

    // Rotation matrices for each axis
    Eigen::Matrix3d Rx;
    Rx = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());

    Eigen::Matrix3d Ry;
    Ry = Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY());

    Eigen::Matrix3d Rz;
    Rz = Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ());

    // Combined rotation matrix
    rot = Rz * Ry * Rx;
}

void EPnPRandomTrans(Eigen::Vector3d& trans) {
    double x = RandomUniform(5, 15);
    double y = RandomUniform(5, 15);
    double z = RandomUniform(20, 50);

    trans << x, y, z;
}

//////////////////////////////////
// Helper functions for CVLab Data
//////////////////////////////////
void SetIntrinsic(std::string calib_path, Eigen::Matrix3d& calib_mat) {
    if(calib_path.empty())
        calib_mat = Eigen::Matrix3d::Identity();
    else
        ReadCVLabCalib(calib_path, calib_mat);
}

void ReadCVLabCalib(std::string calib_path, Eigen::Matrix3d& calib_mat) {
    std::vector<std::vector<double>> cali_infos;
    std::ifstream calis(calib_path);
    std::string line;

    while(std::getline(calis, line)) {
        std::istringstream iss(line);
        // the first iterator process the "line", the second is the dummy one as
        // an end iterator
        std::vector<std::string> curr_line{std::istream_iterator<std::string>{iss}, 
                                           std::istream_iterator<std::string>{}};

        std::vector<double> curr_cali;
        for(auto s: curr_line){
            curr_cali.push_back(std::stod(s));
        }
        cali_infos.push_back(curr_cali);

    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            calib_mat(i, j) = cali_infos[i][j];
        }
    }
}

void ReadCVLab3D(std::string point3d_path, std::vector<Eigen::Vector3d>& points3D) {
    std::vector<std::vector<double>> point3d_list;
    std::ifstream point_file(point3d_path);
    std::string line;

    while(std::getline(point_file, line)) {
        std::istringstream iss(line);
        // the first iterator process the "line", the second is the dummy one as
        // an end iterator
        std::vector<std::string> curr_line{std::istream_iterator<std::string>{iss}, 
                                           std::istream_iterator<std::string>{}};

        std::vector<double> curr_point;
        for(auto s: curr_line){
            curr_point.push_back(std::stod(s));
        }
        point3d_list.push_back(curr_point);
    }

    for(auto p: point3d_list) {
        Eigen::Map<Eigen::Vector3d> curr_eigen(p.data());
        points3D.push_back(curr_eigen);
    }
}

void ReadCVLab2D(std::string point2d_path, std::vector<Eigen::Vector2d>& points2D) {
    std::vector<std::vector<double>> point2d_list;
    std::ifstream point_file(point2d_path);
    std::string line;

    while(std::getline(point_file, line)) {
        std::istringstream iss(line);
        // the first iterator process the "line", the second is the dummy one as
        // an end iterator
        std::vector<std::string> curr_line{std::istream_iterator<std::string>{iss}, 
                                           std::istream_iterator<std::string>{}};

        std::vector<double> curr_point;
        for(auto s: curr_line){
            curr_point.push_back(std::stod(s));
        }
        point2d_list.push_back(curr_point);
    }

    for(auto p: point2d_list) {
        Eigen::Map<Eigen::Vector2d> curr_eigen(p.data());
        points2D.push_back(curr_eigen);
    }
}