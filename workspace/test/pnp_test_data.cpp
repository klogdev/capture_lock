#include <Eigen/Core>
#include <memory>

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <sstream>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
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
        // Handle unsupported types
        default:
            return nullptr;
    }
}

void COLMAPTestData::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<Eigen::Vector3d>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    Eigen::Matrix3d K;
    SetIntrinsic(file_path, K);

    points3D.emplace_back(1, 1, 1);
    points3D.emplace_back(0, 1, 1);
    points3D.emplace_back(3, 1.0, 4);
    points3D.emplace_back(3, 1.1, 4);
    points3D.emplace_back(3, 1.2, 4);
    points3D.emplace_back(3, 1.3, 6);
    points3D.emplace_back(3, 1.4, 5);
    points3D.emplace_back(2, 1, 7);

    for (double qx = 0; qx < 0.5; qx += 0.2) {
        for (double tx = 0; tx < 0.4; tx += 0.1) {
            std::vector<Eigen::Vector2d> curr_points2D;
            const colmap::SimilarityTransform3 orig_tform(1, Eigen::Vector4d(1, qx, 0, 0),
                                                        Eigen::Vector3d(tx, 0, 0));

            // Project points to camera coordinate system
            // here we project the original 3D points
            for (size_t i = 0; i < points3D.size(); i++) {
                Eigen::Vector3d point3D_camera = points3D[i];
                std::cout << "check camera space point: " << std::endl;
                std::cout << point3D_camera << std::endl;
                orig_tform.TransformPoint(&point3D_camera);
                curr_points2D.push_back(point3D_camera.hnormalized());
            }

            points2D.push_back(curr_points2D);
            composed_extrinsic.push_back(orig_tform.Matrix().topLeftCorner<3, 4>());
        }
    }
}

void BoxCornerCameraDistTestData::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                                           std::vector<Eigen::Vector3d>& points3D,
                                           std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {

    Eigen::Matrix3d K;
    SetIntrinsic(file_path, K);
        
    points3D = {
    Eigen::Vector3d(-5, -5, -5),
    Eigen::Vector3d(-5, -5, 5),
    Eigen::Vector3d(-5,  5, -5),
    Eigen::Vector3d(-5,  5, 5),
    Eigen::Vector3d( 5, -5, -5),
    Eigen::Vector3d( 5, -5, 5),
    Eigen::Vector3d( 5,  5, -5),
    Eigen::Vector3d( 5,  5, 5) };

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
            std::cout << "flag before iter is: " << valid_proj << std::endl;

            for (size_t i = 0; i < points3D.size(); i++) {
                Eigen::Vector3d point3D_camera = points3D[i];
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
                composed_extrinsic.push_back(orig_tform.Matrix().topLeftCorner<3, 4>());
            }
        }
    }
}

void CVLabTestData::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                             std::vector<Eigen::Vector3d>& points3D,
                             std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    std::string path_2d = file_path + "2d.txt";
    std::string path_3d = file_path + "3d.txt";
    std::string path_k = file_path + "k.txt";

    Eigen::Matrix3d K;
    SetIntrinsic(path_k, K); // load intrinsic to normalize 2d points
    Eigen::Matrix3d k_inv = K.inverse();

    std::vector<Eigen::Vector2d> one_set_2d; // 2d pixel points vector
    ReadCVLab2D(path_2d, one_set_2d);
    std::vector<Eigen::Vector2d> normalized_2d; // one set of normalized 2d points

    for(const auto& p : one_set_2d) {
        Eigen::Vector3d homogeneousPoint(p[0], p[1], 1.0);
        Eigen::Vector3d homo_point = k_inv*homogeneousPoint;
        Eigen::Vector2d normalized(homo_point[0], homo_point[1]);
        normalized_2d.push_back(normalized);
    }
    points2D.push_back(normalized_2d);

    ReadCVLab3D(path_3d, points3D);

    Eigen::Matrix3x4d dummy_gt = Eigen::Matrix3x4d::Zero();
    composed_extrinsic.push_back(dummy_gt);
}

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