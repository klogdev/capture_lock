#include <Eigen/Core>
#include <memory>

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
        // Handle unsupported types
        default:
            return nullptr;
    }
}

void COLMAPTestData::generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<Eigen::Vector3d>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const {
    points3D.emplace_back(1, 1, 1);
    points3D.emplace_back(0, 1, 1);
    points3D.emplace_back(3, 1.0, 4);
    points3D.emplace_back(3, 1.1, 4);
    points3D.emplace_back(3, 1.2, 4);
    points3D.emplace_back(3, 1.3, 4);
    points3D.emplace_back(3, 1.4, 4);
    points3D.emplace_back(2, 1, 7);

    for (double qx = 0; qx < 0.4; qx += 0.2) {
        for (double tx = 0; tx < 0.5; tx += 0.1) {
            std::vector<Eigen::Vector2d> curr_points2D;
            const colmap::SimilarityTransform3 orig_tform(1, Eigen::Vector4d(1, qx, 0, 0),
                                                        Eigen::Vector3d(tx, 0, 0));

            // Project points to camera coordinate system
            // here we project the original 3D points
            for (size_t i = 0; i < points3D.size(); i++) {
                Eigen::Vector3d point3D_camera = points3D[i];
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
        
    points3D = {
    Eigen::Vector3d(-5, -5, -5),
    Eigen::Vector3d(-5, -5,  5),
    Eigen::Vector3d(-5,  5, -5),
    Eigen::Vector3d(-5,  5,  5),
    Eigen::Vector3d( 5, -5, -5),
    Eigen::Vector3d( 5, -5,  5),
    Eigen::Vector3d( 5,  5, -5),
    Eigen::Vector3d( 5,  5,  5) };

    int num_rot = 3;
    double tz_min = 1.5;
    double tz_max = 3.5;

    for(int i = 0; i < num_rot; i++) {
        Eigen::Vector4d curr_rot = GenRandomRot();
        for(double tz = tz_min; tz <= tz_max; tz += 1) {
            std::vector<Eigen::Vector2d> curr_points2D;
            const colmap::SimilarityTransform3 orig_tform(1, curr_rot,
                                                Eigen::Vector3d(5, 5, tz/10));

            // Project points to camera coordinate system
            // here we project the original 3D points
            for (size_t i = 0; i < points3D.size(); i++) {
                Eigen::Vector3d point3D_camera = points3D[i];
                orig_tform.TransformPoint(&point3D_camera);
                std::cout << "check camera space point: " << std::endl;
                std::cout << point3D_camera << std::endl;
                curr_points2D.push_back(point3D_camera.hnormalized());
            }

            points2D.push_back(curr_points2D);
            composed_extrinsic.push_back(orig_tform.Matrix().topLeftCorner<3, 4>());
        }
    }
}


