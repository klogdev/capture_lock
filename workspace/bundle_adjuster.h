#include <Eigen/Core>
#include <ceres/ceres.h>

#include "optim/bundle_adjustment.h"

class BundleAdjust_{
    public:
        BundleAdjust_(const colmap::BundleAdjustmentOptions& options,
                 const colmap::BundleAdjustmentConfig& config);

        bool Solve();

        const ceres::Solver::Summary& Summary() const;

    private:
        void SetUp(ceres::LossFunction* loss_function);

        void AddImageToProblem(const colmap::image_t image_id,
                         ceres::LossFunction* loss_function);

        void AddPointToProblem(const colmap::point3D_t point3D_id,
                                ceres::LossFunction* loss_function);
        
    protected:
        const colmap::BundleAdjustmentOptions options_;
        colmap::BundleAdjustmentConfig config_;
        std::unique_ptr<ceres::Problem> problem_;
        ceres::Solver::Summary summary_;
        std::unordered_map<point3D_t, size_t> point3D_num_observations_;
}