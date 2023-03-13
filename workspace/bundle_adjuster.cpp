#include "bundle_adjuster.h"
#include <Eigen/Core>
#include <ceres/ceres.h>

BundleAdjust_::BundleAdjust_(const colmap::BundleAdjustmentOptions& options,
                               const colmap::BundleAdjustmentConfig& config)
    : options_(options), config_(config) {}

BundleAdjust_::Solve(){
    problem_ = std::make_unique<ceres::Problem>();
    //default loss in options_: trivial loss
    ceres::LossFunction* loss_function = options_.CreateLossFunction();
    SetUp(loss_function);

    ceres::Solver::Options solver_options = options_.solver_options;

}