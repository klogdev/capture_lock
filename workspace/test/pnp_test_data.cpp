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
        case GeneratorType::COLMAP:
            return std::make_unique<COLMAPTestData>(COLMAPTestData());
        case GeneratorType::BoxDz:
            return std::make_unique<BoxCornerCameraDistTestData>(BoxCornerCameraDistTestData());
        case GeneratorType::CVLab:
            return std::make_unique<CVLabTestData>(CVLabTestData());
        case GeneratorType::EPnPdZ:
            return std::make_unique<BoxCornerEPnPTestData>(BoxCornerEPnPTestData());
        // Handle unsupported types
        default:
            return nullptr;
    }
}

void COLMAPTestData::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                              std::vector<std::vector<Eigen::Vector3d>>& points3D,
                              std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    std::vector<Eigen::Vector3d> one_points3D;
    one_points3D.emplace_back(1, 1, 1);
    one_points3D.emplace_back(0, 1, 1);
    one_points3D.emplace_back(3, 1.0, 4);
    one_points3D.emplace_back(3, 1.1, 4);
    one_points3D.emplace_back(3, 1.2, 4);
    one_points3D.emplace_back(3, 1.3, 6);
    one_points3D.emplace_back(3, 1.4, 5);
    one_points3D.emplace_back(2, 1, 7);

    for (double qx = 0; qx < 0.5; qx += 0.2) {
        for (double tx = 0; tx < 0.4; tx += 0.1) {
            std::vector<Eigen::Vector2d> curr_points2D;
            const colmap::SimilarityTransform3 orig_tform(1, Eigen::Vector4d(1, qx, 0, 0),
                                                        Eigen::Vector3d(tx, 0, 0));

            // Project points to camera coordinate system
            // here we project the original 3D points
            for (size_t i = 0; i < one_points3D.size(); i++) {
                Eigen::Vector3d point3D_camera = one_points3D[i];
                std::cout << "check camera space point: " << std::endl;
                std::cout << point3D_camera << std::endl;
                orig_tform.TransformPoint(&point3D_camera);
                curr_points2D.push_back(point3D_camera.hnormalized());
            }

            points2D.push_back(curr_points2D);
            points3D.push_back(one_points3D);
            composed_extrinsic.push_back(orig_tform.Matrix().topLeftCorner<3, 4>());
        }
    }
}

// void BoxCornerEPnPTestData::IntrinsicSetter() {
//     intrinsic << 800, 0, 320,
//                  0, 800, 240,
//                  0, 0, 1;
// }

void BoxCornerEPnPTestData::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                     std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                     std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    // IntrinsicSetter(); // set the intrinsic by default values
    // Define the ranges for x, y, and z
    double x_values[2] = {-2, 2};
    double y_values[2] = {-2, 2};
    double z_values[2] = {4, 8};

    std::vector<Eigen::Vector3d> camera_space_points;
    // Generate the corners
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                camera_space_points.push_back(Eigen::Vector3d(x_values[i], y_values[j], z_values[k]));
            }
        }
    }

    // project corners as 2d points
    std::vector<Eigen::Vector2d> one_set_2d;
    for(int i = 0; i < points3D.size(); i++) {
        Eigen::Vector3d curr_homo = camera_space_points[i];
        one_set_2d.push_back(Eigen::Vector2d(curr_homo.x()/curr_homo.z(), curr_homo.y()/curr_homo.z()));
    }

    int num_rot = 3;
    double tz_min = 2.5;
    double tz_max = 5.5;

    for(int i = 0; i < num_rot; i++) {
        Eigen::Matrix3d curr_rot;
        EPnPRandomRot(curr_rot);
        for(double tz = tz_min; tz <= tz_max; tz += 1) {
            std::vector<Eigen::Vector3d> curr_points3d;
            Eigen::Vector3d curr_trans = Eigen::Vector3d(5, 5, tz/10);
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot),
                                                curr_trans);
            // generate scene points
            for (size_t i = 0; i < camera_space_points.size(); i++) {
                Eigen::Vector3d point3D_world = camera_space_points[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from a single camera points set
            points2D.push_back(one_set_2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot.transpose(); 
            curr_gt.col(3) = -curr_rot.transpose()*curr_trans; 
            composed_extrinsic.push_back(curr_gt);
        }
    }
}

void BoxCornerCameraDistTestData::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                           std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                           std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    
    std::vector<Eigen::Vector3d> one_points3D = {
    Eigen::Vector3d(-5, -5, -5),
    Eigen::Vector3d(-5, -5, 5),
    Eigen::Vector3d(-5,  5, -5),
    Eigen::Vector3d(-5,  5, 5),
    Eigen::Vector3d( 5, -5, -5),
    Eigen::Vector3d( 5, -5, 5),
    Eigen::Vector3d( 5,  5, -5),
    Eigen::Vector3d( 5,  5, 5) 
    };

    int num_rot = 3;
    double tz_min = 2.5;
    double tz_max = 5.5;

    for(int i = 0; i < num_rot; i++) {
        Eigen::Vector4d curr_rot = GenRandomRot();
        for(double tz = tz_min; tz <= tz_max; tz += 1) {
            std::vector<Eigen::Vector2d> curr_points2D;
            const colmap::SimilarityTransform3 orig_tform(1, curr_rot,
                                                Eigen::Vector3d(5, 5, tz/10));

            // Project points to camera coordinate system
            // here we project the original 3D points
            // we use flag to skip the point w/ negative depth
            bool valid_proj = true;

            for (size_t i = 0; i < one_points3D.size(); i++) {
                Eigen::Vector3d point3D_camera = one_points3D[i];
                orig_tform.TransformPoint(&point3D_camera);
                if(point3D_camera.z() <= 0) {
                    valid_proj = false;
                    break;
                }
                std::cout << "check camera space point: " << std::endl;
                std::cout << point3D_camera << std::endl;
                curr_points2D.push_back(point3D_camera.hnormalized());
            }

            std::cout << "flag after iter is: " << valid_proj << std::endl;
            if (valid_proj == true) {
                std::cout << "enter this if statment" << std::endl;
                points2D.push_back(curr_points2D);
                points3D.push_back(one_points3D);
                composed_extrinsic.push_back(orig_tform.Matrix().topLeftCorner<3, 4>());
            }
        }
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

///////////////////////
// Helper function for EPnP synthetic data
//////////////////////
double EPnPRandom(double x_ini, double x_end) {
    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 gen(rd()); // Seed the generator
    std::uniform_real_distribution<> distr(x_ini, x_end); // Define the range

    double x = distr(gen); // Generate a random number

    return x;
}

void EPnPRandomRot(Eigen::Matrix3d& rot) {
    double alpha = EPnPRandom(0, 45);
    double beta = EPnPRandom(0, 45);
    double gamma = EPnPRandom(0, 45);

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

///////////////////////
// Helper functions for CVLab Data
//////////////////////
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