#include "bundle_adjuster.h"
#include <Eigen/Core>
#include <ceres/ceres.h>
#include "base/image.h"
#include "base/point2d.h"
#include "base/camera_models.h"
#include "base/cost_functions.h"

//config: include all images&point should be setup in priori
BundleAdjust_::BundleAdjust_(const colmap::BundleAdjustmentOptions& options,
                            const colmap::BundleAdjustmentConfig& config)
    : options_(options), config_(config) {}

bool BundleAdjust_::Solve(){
    problem_ = std::make_unique<ceres::Problem>();
    //default loss in options_: trivial loss
    ceres::LossFunction* loss_function = options_.CreateLossFunction();
    SetUp(loss_function);

    ceres::Solver::Options solver_options = options_.solver_options;
    return true;
}

void BundleAdjust_::SetUp(ceres::LossFunction* loss_function){
    for (const colmap::image_t image_id : config_.Images()) {
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

    const bool constant_pose =
      !options_.refine_extrinsics || config_.HasConstantPose(image_id);

    size_t num_observations = 0;
    for (const colmap::Point2D& point_2d: image.Points2D()){
        if (!point_2d.HasPoint3D()){
            continue;
        }
        num_observations += 1;
        point3D_num_observations_[point_2d.Point3DId()] += 1;

        colmap::Point3D& curr_3d = global_3d_map[point2D.Point3DId()];
        assert(curr_3d.Track().Length() > 1); //must have more than 1 obs, to have enough DoF

        ceres::CostFunction* cost_function = nullptr;
        //constant pose init by both 2d point and extrinsic, only intrinsic/3d point as parametes
        if (constant_pose) {
            switch (camera.ModelId()) {
        #define CAMERA_MODEL_CASE(CameraModel)                                 \
        case colmap::CameraModel::kModelId:                                          \
            cost_function =                                                    \
                colmap::BundleAdjustmentConstantPoseCostFunction<colmap::CameraModel>::Create( \
                    image.Qvec(), image.Tvec(), point2D.XY());                 \
            break;

                CAMERA_MODEL_SWITCH_CASES

        #undef CAMERA_MODEL_CASE
            }

            problem_->AddResidualBlock(cost_function, loss_function,
                                        curr_3d.XYZ().data(), camera_params_data);
            } 
        else{
            switch (camera.ModelId()) {
                #define CAMERA_MODEL_CASE(CameraModel)                                   \
                case colmap::CameraModel::kModelId:                                            \
                    cost_function =                                                      \
                        colmap::BundleAdjustmentCostFunction<CameraModel>::Create(point2D.XY()); \
                    break;

                        CAMERA_MODEL_SWITCH_CASES

                #undef CAMERA_MODEL_CASE
            }
        problem_->AddResidualBlock(cost_function, loss_function, qvec_data,
                                    tvec_data, curr_3d.XYZ().data(),
                                    camera_params_data); 
        }
    }
    
    if (!constant_pose){
        colmap::SetQuaternionManifold(problem_.get(), qvec_data);
    }
    
}

void BundleAdjust_::AddPointToProblem(const colmap::point3D_t point3D_id,
                                        colmap::Camera& camera,
                                        std::unordered_map<int,colmap::Image>& global_image_map,
                                        std::unordered_map<int,colmap::Point3D>& global_3d_map,
                                        ceres::LossFunction* loss_function){
    colmap::Point3D curr_3d = global_3d_map[point3D_id];

    if (point3D_num_observations_[point3D_id] == curr_3d.Track().Length())
        return;

    for (const auto& track_el: curr_3d.Track().Elements()){
        if (config_.HasImage(track_el.image_id)) {
            continue;
        }
        point3D_num_observations_[point3D_id] += 1;

        colmap::Image track_image = global_image_map[track_el.image_id];
        const colmap::Point2D track_point_2d = track_image.Point2D(track_el.point2D_idx);

        ceres::CostFunction* cost_function = nullptr;

        switch (camera.ModelId()) {
            #define CAMERA_MODEL_CASE(CameraModel)                                 \
            case CameraModel::kModelId:                                          \
                cost_function =                                                    \
                    colmap::BundleAdjustmentConstantPoseCostFunction<CameraModel>::Create( \
                        track_image.Qvec(), track_image.Tvec(), track_point_2d.XY());                 \
                break;

                CAMERA_MODEL_SWITCH_CASES

            #undef CAMERA_MODEL_CASE
                }
    problem_->AddResidualBlock(cost_function, loss_function,
                               curr_3d.XYZ().data(), camera.ParamsData());
    }
}

void BundleAdjust_::ParameterizePoints(
                    std::unordered_map<int,colmap::Point3D>& global_point3d_map){
    for (const auto elem: point3D_num_observations_){
        colmap::Point3D& curr_3d = global_point3d_map[elem.first];
        if (curr_3d.Track().Length() > elem.second){
            problem_->SetParameterBlockConstant(curr_3d.XYZ().data());
        }
    }
}
