#include <Eigen/Core>

#include "base/camera.h"
#include "base/camera_models.h"
#include "base/image.h"
#include "base/point3d.h"
#include "base/point2d.h"

#include "base/projection.h"

void CheckTUMResidual(colmap::Image& curr_image, colmap::Camera& camera,
                      std::unordered_map<int, colmap::Point3D>& global_3d_map) {
    int curr_id = curr_image.ImageId();
    Eigen::Vector4d qvec = curr_image.Qvec();
    Eigen::Vector3d tvec = curr_image.Tvec();

    for(const colmap::Point2D& p: curr_image.Points2D()) {
        int id_3d = p.Point3DId();
        double curr_res = colmap::CalculateSquaredReprojectionError(p.XY(),
                                                                    global_3d_map[id_3d].XYZ(),
                                                                    qvec, tvec, camera);
        // std::cout << "3D point " << id_3d << "'s reprojection error on image " << curr_id 
        // << " is " << curr_res << std::endl;
    }
}