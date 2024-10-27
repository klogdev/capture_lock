#include <Eigen/Core>

#include "base/camera.h"
#include "base/camera_models.h"
#include "base/image.h"
#include "base/point3d.h"
#include "base/point2d.h"
#include "base/pose.h"
#include "base/projection.h"

void CheckTUMResidual(colmap::Image* curr_image, colmap::Camera& camera,
                      std::unordered_map<int, colmap::Point3D>& global_3d_map) {
    int curr_id = curr_image->ImageId();
    Eigen::Vector4d qvec = curr_image->Qvec();
    Eigen::Vector3d tvec = curr_image->Tvec();

    for(const colmap::Point2D& p: curr_image->Points2D()) {
        int id_3d = p.Point3DId();
        double curr_res = colmap::CalculateSquaredReprojectionError(p.XY(),
                                                                    global_3d_map[id_3d].XYZ(),
                                                                    qvec, tvec, camera);
        const Eigen::Vector3d proj_point3D =
            colmap::QuaternionRotatePoint(qvec, global_3d_map[id_3d].XYZ()) + tvec;

        // Check that point is infront of camera.
        if (proj_point3D.z() < std::numeric_limits<double>::epsilon()) {
            std::cout << std::numeric_limits<double>::max() << std::endl;
        }

        const Eigen::Vector2d proj_point2D =
            camera.WorldToImage(proj_point3D.hnormalized());
        std::cout << "curr 2d is: " << p.XY().transpose() << std::endl;
        std::cout << "curr projected 2d is: " << proj_point2D.transpose() << std::endl;
        std::cout << "3D point " << id_3d << "'s reprojection error on image " << curr_id 
        << " is " << curr_res << std::endl;
    }
}