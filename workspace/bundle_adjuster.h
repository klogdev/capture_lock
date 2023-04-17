#include <Eigen/Core>
#include <ceres/ceres.h>
#include "base/camera.h"

#include "optim/bundle_adjustment.h"

class BundleAdjust_{
    public:
        BundleAdjust_(const colmap::BundleAdjustmentOptions& options,
                 const colmap::BundleAdjustmentConfig& config);

        bool Solve(colmap::Camera& camera,
                   std::unordered_map<int,colmap::Image>& global_image_map,
                   std::unordered_map<int,colmap::Point3D>& global_3d_map);

        const ceres::Solver::Summary& Summary() const;

        std::vector<double> RetrieveParas(const int parameter_idx);

    private:
        void SetUp(ceres::LossFunction* loss_function, colmap::Camera& camera,
                   std::unordered_map<int,colmap::Image>& global_image_map,
                   std::unordered_map<int,colmap::Point3D>& global_3d_map);

        void AddImageToProblem(const colmap::image_t image_id,
                        colmap::Camera& camera,
                        std::unordered_map<int,colmap::Image>& global_image_map,
                        std::unordered_map<int,colmap::Point3D>& global_3d_map,
                        ceres::LossFunction* loss_function);

        void AddPointToProblem(const colmap::point3D_t point3D_id,
                                colmap::Camera& camera,
                                std::unordered_map<int,colmap::Image>& global_image_map,
                                std::unordered_map<int,colmap::Point3D>& global_3d_map,
                                ceres::LossFunction* loss_function);
        
    protected:
        void ParameterizePoints(std::unordered_map<int,colmap::Point3D>& global_point3d_map);

        const colmap::BundleAdjustmentOptions options_;
        colmap::BundleAdjustmentConfig config_;
        std::unique_ptr<ceres::Problem> problem_;
        ceres::Solver::Summary summary_;
        std::unordered_map<colmap::point3D_t, size_t> point3D_num_observations_;
};