#include <Eigen/Core>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "base/camera_models.h"

class BACostFxn{
    public:
        explicit BACostFxn(const Eigen::Vector2d& point_2d)
            :obs_x(point_2d(0)), obs_y(point_2d(1)){}

        //num_res, list[dim_paras]
        static ceres::CostFunction* Create(const Eigen::Vector2d& point_2d){
            return new ceres::AutoDiffCostFunction<BACostFxn,2,4,3,3,4>(
                new BACostFxn(point_2d)
            );
        }
        //4,3,3,4 => qvec, tvec, point_3d, simple pinhole camera params
        template<typename T>
        bool operator()(const T* const qvec, const T* const tvec,
                  const T* const point_3d, const T* const camera_params,
                  T* residuals) const{
            // Rotate and translate.
            T projection[3];
            
            ceres::UnitQuaternionRotatePoint(qvec, point_3d, projection);
            projection[0] += tvec[0];
            projection[1] += tvec[1];
            projection[2] += tvec[2];

            // Project to image plane.
            projection[0] /= projection[2];
            projection[1] /= projection[2];

            // Distort and transform to pixel space.
            colmap::SimpleRadialCameraModel::WorldToImage(camera_params, projection[0], projection[1],
                                    &residuals[0], &residuals[1]);

            // Re-projection error.
            residuals[0] -= T(obs_x);
            residuals[1] -= T(obs_y);

            
            if (isnan(residuals[0]) || isnan(residuals[1])){
                std::cout << "Error: NaN residual value encountered in Cost Fxn." << std::endl;
                std::cout << "obs: " << obs_x << ", " << obs_y << std::endl;
                std::cout << "Cost Fxn Proj: " << projection[0] << 
                ", " << projection[1] << std::endl;
                for(int i = 0; i < 4; i++){
                    std::cout << "Camera parameters: " << camera_params[i] << std::endl;
                }
                for(int i = 0; i < 4; i++){
                    std::cout << "qvec: " << qvec[i] << std::endl;
                }
                for(int i = 0; i < 3; i++){
                    std::cout << "tvec: " << tvec[i] << std::endl;
                }
            }

            return true;
        }
    
    private:
        const double obs_x;
        const double obs_y;
};

class BAConstPoseCostFxn {
 public:
    BAConstPoseCostFxn(const Eigen::Vector4d& qvec,
                        const Eigen::Vector3d& tvec,
                        const Eigen::Vector2d& point_2d)
      : qw_(qvec(0)),
        qx_(qvec(1)),
        qy_(qvec(2)),
        qz_(qvec(3)),
        tx_(tvec(0)),
        ty_(tvec(1)),
        tz_(tvec(2)),
        obs_x(point_2d(0)),
        obs_y(point_2d(1)) {}

    static ceres::CostFunction* Create(const Eigen::Vector4d& qvec,
                                       const Eigen::Vector3d& tvec,
                                       const Eigen::Vector2d& point_2d){
        return (new ceres::AutoDiffCostFunction<
                BAConstPoseCostFxn, 2, 3, 4>(
            new BAConstPoseCostFxn(qvec, tvec, point_2d)));
    }

    template <typename T>
    bool operator()(const T* const point_3d, const T* const camera_params,
                    T* residuals) const {
        const T qvec[4] = {T(qw_), T(qx_), T(qy_), T(qz_)};

        // Rotate and translate.
        T projection[3];
        ceres::UnitQuaternionRotatePoint(qvec, point_3d, projection);
        projection[0] += T(tx_);
        projection[1] += T(ty_);
        projection[2] += T(tz_);

        // Project to image plane.
        projection[0] /= projection[2];
        projection[1] /= projection[2];

        // Distort and transform to pixel space.
        colmap::SimpleRadialCameraModel::WorldToImage(camera_params, projection[0], projection[1],
                                &residuals[0], &residuals[1]);

        // Re-projection error.
        residuals[0] -= T(obs_x);
        residuals[1] -= T(obs_y);

        // std::cout << "Const Cost Fxn Residual: " << residuals[0] << 
        //         ", " << residuals[1] << std::endl;
        if (isnan(residuals[0]) || isnan(residuals[1]))
                std::cout << "Error: NaN residual value encountered in Const Cost Fxn." << std::endl;

        return true;
    }

    private:
    const double qw_;
    const double qx_;
    const double qy_;
    const double qz_;
    const double tx_;
    const double ty_;
    const double tz_;
    const double obs_x;
    const double obs_y;
};