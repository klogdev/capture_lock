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
        case GeneratorType::CVLab:
            return std::make_unique<CVLabTestData>(CVLabTestData());
        case GeneratorType::EPnPdZ:
            return std::make_unique<BoxCornerEPnPTestDataDz>(BoxCornerEPnPTestDataDz());
        case GeneratorType::EPnPdY:
            return std::make_unique<BoxCornerEPnPTestDataDy>(BoxCornerEPnPTestDataDy());
        case GeneratorType::RandomDz:
            return std::make_unique<BoxRandomEPnPTestDataDz>(BoxRandomEPnPTestDataDz());
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
                orig_tform.TransformPoint(&point3D_camera);
                curr_points2D.push_back(point3D_camera.hnormalized());
            }

            points2D.push_back(curr_points2D);
            points3D.push_back(one_points3D);
            composed_extrinsic.push_back(orig_tform.Matrix().topLeftCorner<3, 4>());
        }
    }
}

// set default values for static members
double BoxCornerEPnPTestDataDz::sigma = 0.0003;
void BoxCornerEPnPTestDataDz::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                       std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                       std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    // set the default intrinsic matrix and Box corners
    Eigen::Matrix3d k;
    std::vector<Eigen::Vector3d> camera_space_points;
    EPnPBoxCorner(k, camera_space_points);

    Eigen::Matrix3d k_inv = k.inverse();

    std::cout << "check sigma used in generator: " << sigma << std::endl;

    int num_samples = 500;
    std::vector<std::vector<Eigen::Vector2d>> all_2ds(num_samples, std::vector<Eigen::Vector2d>());
    // project corners as 2d points, then add noise, and backprojected for each set
    for(int j = 0; j < num_samples; j++) {
        std::vector<Eigen::Vector2d> one_set_2d;
        for(int i = 0; i < camera_space_points.size(); i++) {
            Eigen::Vector3d curr_camera_pt = camera_space_points[i];
            Eigen::Vector3d curr_projected = k*curr_camera_pt;
            double curr_pix_x = curr_projected.x()/curr_projected.z();
            double curr_pix_y = curr_projected.y()/curr_projected.z();
            Eigen::Vector3d noised_image_pt = 
                            Eigen::Vector3d(RandomGaussian(curr_pix_x, sigma),
                                            RandomGaussian(curr_pix_y, sigma),
                                            1);
            Eigen::Vector3d noised_camera_pt = k_inv*noised_image_pt; // do backprojection
            one_set_2d.push_back(Eigen::Vector2d(noised_camera_pt.x()/noised_camera_pt.z(),
                                        noised_camera_pt.y()/noised_camera_pt.z()));
        }
        std::cout << "size of " << j << "'s 2d set is: " << one_set_2d.size() << std::endl;
        all_2ds[j] = one_set_2d;
    }

    std::cout << "check the 1st set of backprojected camera points: " << std::endl;
    for(auto& p: all_2ds[0]) {
        std::cout << p << std::endl;
    }

    double d_min = 35;
    double d_max = 35;

    for(double d = d_min; d <= d_max; d += 1) {
        Eigen::Matrix3d curr_rot;
        EPnPRandomRot(curr_rot);
        for(int i = 0; i < num_samples; i++) {
            Eigen::Matrix3d curr_rot;
            EPnPRandomRot(curr_rot);
            std::vector<Eigen::Vector3d> curr_points3d;
            Eigen::Vector3d curr_trans = Eigen::Vector3d(0, 0, d);
            const colmap::SimilarityTransform3 orig_tform(1, colmap::RotationMatrixToQuaternion(curr_rot),
                                                curr_trans);
            // generate scene points from non-noised camera points
            for (size_t i = 0; i < camera_space_points.size(); i++) {
                Eigen::Vector3d point3D_world = camera_space_points[i];
                orig_tform.TransformPoint(&point3D_world);
                curr_points3d.push_back(point3D_world);
            }
            // EPnP generate all scene points from a single camera points set
            points2D.push_back(all_2ds[i]);
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
    std::vector<Eigen::Vector3d> camera_space_points;
    EPnPBoxCorner(k, camera_space_points);

    Eigen::Matrix3d k_inv = k.inverse();

    int num_samples = 500;
    std::vector<std::vector<Eigen::Vector2d>> all_2ds(num_samples, std::vector<Eigen::Vector2d>());

    // project corners as 2d points
    for(int j = 0; j < num_samples; j++) {
        std::vector<Eigen::Vector2d> one_set_2d;
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
        all_2ds[j] = one_set_2d;
    }

    double d_min = 15;
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
            points2D.push_back(all_2ds[i]);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot.transpose(); 
            curr_gt.col(3) = -curr_rot.transpose()*curr_trans; 
            composed_extrinsic.push_back(curr_gt);
        }
    }
}

// set default values for static members
double BoxRandomEPnPTestDataDz::sigma = 0.05;
void BoxRandomEPnPTestDataDz::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                     std::vector<std::vector<Eigen::Vector3d>>& points3D,
                                     std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    // set the default intrinsic matrix and Box corners
    Eigen::Matrix3d k;
    std::vector<Eigen::Vector3d> camera_space_points;
    int num_pts = 25;
    EPnPInsideRand(k, camera_space_points, num_pts);

    Eigen::Matrix3d k_inv = k.inverse();

    // project corners as 2d points
    std::vector<Eigen::Vector2d> one_set_2d;
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

    std::cout << "check backprojected camera points: " << std::endl;
    for(auto& p: one_set_2d) {
        std::cout << p << std::endl;
    }

    int num_rot = 500;
    double d_min = 15;
    double d_max = 500;

    for(int i = 0; i < num_rot; i++) {
        Eigen::Matrix3d curr_rot;
        EPnPRandomRot(curr_rot);
        for(double d = d_min; d <= d_max; d += 10) {
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
            points2D.push_back(one_set_2d);
            points3D.push_back(curr_points3d);
            Eigen::Matrix3x4d curr_gt;
            curr_gt.block<3, 3>(0, 0) = curr_rot.transpose(); 
            curr_gt.col(3) = -curr_rot.transpose()*curr_trans; 
            composed_extrinsic.push_back(curr_gt);
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

//////////////////////////////////////////
// Helper function for EPnP synthetic data
/////////////////////////////////////////
void EPnPBoxCorner(Eigen::Matrix3d& k, std::vector<Eigen::Vector3d>& camera_space_points) {
    k << 800, 0, 320,
         0, 800, 240,
         0, 0, 1;

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

void EPnPInsideRand(Eigen::Matrix3d& k, std::vector<Eigen::Vector3d>& camera_space_points,
                    int num_pts) {
    k << 800, 0, 320,
         0, 800, 240,
         0, 0, 1;

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