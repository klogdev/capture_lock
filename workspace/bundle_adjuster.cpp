#include "bundle_adjuster.h"
#include <Eigen/Core>
#include <ceres/ceres.h>
#include "base/image.h"
#include "base/point2d.h"
#include "base/camera_models.h"
#include "base/cost_functions.h"

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
                        std::unordered_map<int,colmap::Point3D>& global_3d_map,
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
        num_observations += 1;
        point3D_num_observations_[point2D.Point3DId()] += 1;

        Point3D& curr_3d = global_3d_map[point2D.Point3DId()];
        assert(curr_3d.Track().Length() > 1); //must have more than 1 obs, to have enough DoF

        switch (camera.ModelId()) {
            #define CAMERA_MODEL_CASE(CameraModel)                                   \
            case CameraModel::kModelId:                                            \
                cost_function =                                                      \
                    BundleAdjustmentCostFunction<CameraModel>::Create(point2D.XY()); \
                break;

                    CAMERA_MODEL_SWITCH_CASES

            #undef CAMERA_MODEL_CASE
      }
      problem_->AddResidualBlock(cost_function, loss_function, qvec_data,
                                 tvec_data, point3D.XYZ().data(),
                                 camera_params_data); 
    }
    colmap::SetQuaternionManifold(problem_.get(), qvec_data);
}