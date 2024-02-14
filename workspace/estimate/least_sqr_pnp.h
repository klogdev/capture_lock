#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>

struct ReprojectionError {
    ReprojectionError(const Eigen::Vector2d& point_2d, const Eigen::Vector3d& point_3d)
        : observed_x(point_2d.x()), observed_y(point_2d.y()),
          point3D_x(point_3d.x()), point3D_y(point_3d.y()), point3D_z(point_3d.z()) {}

    template <typename T>
    bool operator()(const T* const camera_rotation,    // Camera rotation (quaternion)
                    const T* const camera_translation, // Camera translation
                    T* residuals) const {
        
        T p_transformed[3];
        T point_3d[3] = {T(point3D_x), T(point3D_y), T(point3D_z)};
        ceres::UnitQuaternionRotatePoint(camera_rotation, point_3d, p_transformed);

        // Apply translation
        p_transformed[0] += camera_translation[0];
        p_transformed[1] += camera_translation[1];
        p_transformed[2] += camera_translation[2];

        // Project to 2D, here we assume simple pinhole model
        T xp = p_transformed[0] / p_transformed[2];
        T yp = p_transformed[1] / p_transformed[2];

        // Compute residuals
        residuals[0] = xp - T(observed_x);
        residuals[1] = yp - T(observed_y);

        return true;
    }

    private:
        double observed_x, observed_y;
        double point3D_x, point3D_y, point3D_z;
};

/**
 * @brief ceres solver for the least square optimization of PnP
 * can be a sccesive polish step after LHM/DRaM
*/
void LeastSquareSolver(const std::vector<Eigen::Vector2d>& points_2d, 
                       const std::vector<Eigen::Vector3d>& points_3d,
                       Eigen::Vector4d& quat_init, Eigen::Vector3d& trans_init,
                       int max_iters);