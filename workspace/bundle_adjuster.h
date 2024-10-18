#include <Eigen/Core>
#include <ceres/ceres.h>

#include "base/camera.h"
#include "base/camera_models.h"

#include "optim/bundle_adjustment.h"

#include "file_reader/data_types.h"

/**
 * @brief main class to do BA
 * @arg config: include all images&point should be setup in priori
 * options are adopt from colmap's BA option
 * we add data as an extra arg to instantiate the cost fn w/ different
 * camera model
*/
class BundleAdjust_{
    public:
        BundleAdjust_(const colmap::BundleAdjustmentOptions& options,
                      const colmap::BundleAdjustmentConfig& config,
                      const Dataset data);

        bool Solver(colmap::Camera& camera,
                   std::unordered_map<int, colmap::Image>& global_image_map,
                   std::unordered_map<int, colmap::Point3D>& global_3d_map);

        const ceres::Solver::Summary& Summary() const;

    private:
        void SetUp(ceres::LossFunction* loss_function, colmap::Camera& camera,
                   std::unordered_map<int,colmap::Image>& global_image_map,
                   std::unordered_map<int,colmap::Point3D>& global_3d_map);

        void AddImageToProblem(const colmap::image_t image_id,
                        colmap::Camera& camera,
                        std::unordered_map<int, colmap::Image>& global_image_map,
                        std::unordered_map<int, colmap::Point3D>& global_3d_map,
                        ceres::LossFunction* loss_function);

        void AddPointToProblem(const colmap::point3D_t point3D_id,
                                colmap::Camera& camera,
                                std::unordered_map<int,colmap::Image>& global_image_map,
                                std::unordered_map<int,colmap::Point3D>& global_3d_map,
                                ceres::LossFunction* loss_function);
        
    protected:
        void ParameterizePoints(std::unordered_map<int,colmap::Point3D>& global_point3d_map);
        void ParameterizeCameras(colmap::Camera& camera);

        const colmap::BundleAdjustmentOptions options_;
        Dataset dataset_;
        colmap::BundleAdjustmentConfig config_;
        std::unique_ptr<ceres::Problem> problem_;
        ceres::Solver::Summary summary_;
        std::unordered_map<colmap::point3D_t, size_t> point3D_num_observations_;
        std::unordered_set<colmap::camera_t> camera_ids_;
};