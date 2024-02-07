#include <Eigen/Core>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

void COLMAPTestData(std::vector<Eigen::Vector2d>& points2D, 
                    std::vector<Eigen::Vector3d>& points3D,
                    std::vector<Eigen::Matrix3x4d>& composed_extrinsic) {
    points3D.emplace_back(1, 1, 1);
    points3D.emplace_back(0, 1, 1);
    points3D.emplace_back(3, 1.0, 4);
    points3D.emplace_back(3, 1.1, 4);
    points3D.emplace_back(3, 1.2, 4);
    points3D.emplace_back(3, 1.3, 4);
    points3D.emplace_back(3, 1.4, 4);
    points3D.emplace_back(2, 1, 7);

    for (double qx = 0; qx < 0.2; qx += 0.2) {
        for (double tx = 0; tx < 0.2; tx += 0.1) {
            const colmap::SimilarityTransform3 orig_tform(1, Eigen::Vector4d(1, qx, 0, 0),
                                                          Eigen::Vector3d(tx, 0, 0));

            // Project points to camera coordinate system
            // here we project the original 3D points
            std::vector<Eigen::Vector2d> points2D;
            for (size_t i = 0; i < points3D.size(); i++) {
                Eigen::Vector3d point3D_camera = points3D[i];
                orig_tform.TransformPoint(&point3D_camera);
                points2D.push_back(point3D_camera.hnormalized());
            }

            std::cout << "current rotation simulation is: " << qx << std::endl;
            std::cout << "current translation simulation is: " << tx << std::endl;
            std::cout << "current transform is: " << std::endl;
            std::cout << orig_tform.Matrix() << std::endl;
            composed_extrinsic.push_back(orig_tform.Matrix().topLeftCorner<3, 4>());
        }
    }
}

