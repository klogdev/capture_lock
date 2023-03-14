#include "bundle_adjuster.h"
#include <Eigen/Core>
#include <ceres/ceres.h>
#include "base/image.h"
#include "base/point2d.h"

BundleAdjust_::BundleAdjust_(const colmap::BundleAdjustmentOptions& options,
                               const colmap::BundleAdjustmentConfig& config)
    : options_(options), config_(config) {}

BundleAdjust_::Solve(){
    problem_ = std::make_unique<ceres::Problem>();
    //default loss in options_: trivial loss
    ceres::LossFunction* loss_function = options_.CreateLossFunction();
    SetUp(loss_function);

    ceres::Solver::Options solver_options = options_.solver_options;
    return true;
}

BundleAdjust_::SetUp(ceres::LossFunction* loss_function){
    for (const image_t image_id : config_.Images()) {
        AddImageToProblem(image_id, loss_function);
    }
    //need call BA config for preprocess
    for (const auto point3D_id : config_.VariablePoints()) {
        AddPointToProblem(point3D_id, loss_function);
    }
    //skip camera parameterization?
    ParameterizePoints();
}

void BundleAdjust_::AddImageToProblem(const colmap::image_t image_id,
                        colmap::Camera& camera,
                        std::unordered_map<int,colmap::Image>& global_image_map,
                        ceres::LossFunction* loss_function){
    colmap::Image& image = global_image_map[image_id];
    image.NormalizeQvec();

    double* qvec_data = image.Qvec().data();
    double* tvec_data = image.Tvec().data();
    double* camera_params_data = camera.ParamsData();

    size_t num_observations = 0;
    for (const Point2D& point_2d: image.Points2D()){
        if (!point_2d.HasPoint3D()){
            continue;
        }
        
    }
    
}