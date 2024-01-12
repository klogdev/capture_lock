#include "bundle_adjuster.h"
#include "cost_fxn.h"

#include <Eigen/Core>
#include <ceres/ceres.h>
#include "base/image.h"
#include "base/point2d.h"
#include "base/camera_models.h"
#include "base/cost_functions.h"

#include "file_reader/data_types.h"


BundleAdjust_::BundleAdjust_(const colmap::BundleAdjustmentOptions& options,
                             const colmap::BundleAdjustmentConfig& config,
                             const Dataset data)
    : options_(options), config_(config), dataset_(data) {
        // different from colmap, we move init of problem_ 
        // from Solver to the constructor
        problem_ = std::make_unique<ceres::Problem>();
    }

bool BundleAdjust_::Solver(colmap::Camera& camera,
                          std::unordered_map<int,colmap::Image>& global_image_map,
                          std::unordered_map<int,colmap::Point3D>& global_3d_map){
    // default loss in options_: trivial loss
    ceres::LossFunction* loss_function = options_.CreateLossFunction();
    // DEBUGGING:
    std::cout << "curr image set: " << std::endl;
    for (const auto& element : config_.Images()) {
        std::cout << element << " ";
    }
    std::cout << std::endl;

    SetUp(loss_function, camera, global_image_map, global_3d_map);

    ceres::Solver::Options solver_options = options_.solver_options;
    std::cout << "no segfault before BA solver in this set " <<  std::endl;
    ceres::Solve(solver_options, problem_.get(), &summary_);
    std::cout << "no segfault after BA solver in this set " <<  std::endl;

    return true;
}

void BundleAdjust_::SetUp(ceres::LossFunction* loss_function,
                          colmap::Camera& camera,
                          std::unordered_map<int,colmap::Image>& global_image_map,
                          std::unordered_map<int,colmap::Point3D>& global_3d_map){
    // need call BA config for preprocess;
    // the config_ is the private member of BA,
    // and will be processed within the global_bundle
    for (const colmap::image_t image_id : config_.Images()) {
        AddImageToProblem(image_id, camera, global_image_map, 
                          global_3d_map, loss_function);
    }
    
    for (const auto point3D_id : config_.VariablePoints()) {
        AddPointToProblem(point3D_id, camera, global_image_map, 
                          global_3d_map, loss_function);
    }
    // need to set camera parameter block, if it specified as a constant
    ParameterizePoints(global_3d_map);
    ParameterizeCameras(camera);
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

        colmap::Point3D& curr_3d = global_3d_map[point_2d.Point3DId()];
        assert(curr_3d.Track().Length() > 1); // must have more than 1 obs, to have enough DoF

        ceres::CostFunction* cost_function = nullptr;
        // constant pose init by both 2d point and extrinsic, only intrinsic/3d point as parametes
        if (constant_pose) {  
            if (dataset_ == Dataset::Colmap){
                cost_function =                                                    
                    BAConstPoseCostFxn<colmap::SimpleRadialCameraModel>::Create( 
                        image.Qvec(), image.Tvec(), point_2d.XY()); 
            }  
            else if (dataset_ == Dataset::Kitti){
                cost_function =                                                    
                    BAConstPoseCostFxn<colmap::SimplePinholeCameraModel>::Create( 
                        image.Qvec(), image.Tvec(), point_2d.XY()); 
            }              

            problem_->AddResidualBlock(cost_function, loss_function,
                                       curr_3d.XYZ().data(), camera_params_data);
        } 

        else{ 
            if (dataset_ == Dataset::Colmap){                                     
                cost_function = BACostFxn<colmap::SimpleRadialCameraModel>::Create(point_2d.XY()); 
            }
            else if (dataset_ == Dataset::Kitti){
                cost_function = BACostFxn<colmap::SimplePinholeCameraModel>::Create(point_2d.XY()); 
            }
           
            problem_->AddResidualBlock(cost_function, loss_function, qvec_data,
                                       tvec_data, curr_3d.XYZ().data(),
                                       camera_params_data); 
        }
    }
    
    
    // we do not need to register cam id with non-zero observations
    // as we assume all images added has at least 1 observation
    if (!constant_pose){
        colmap::SetQuaternionManifold(problem_.get(), qvec_data);
    }    
}

void BundleAdjust_::AddPointToProblem(const colmap::point3D_t point3D_id,
                                      colmap::Camera& camera,
                                      std::unordered_map<int,colmap::Image>& global_image_map,
                                      std::unordered_map<int,colmap::Point3D>& global_3d_map,
                                      ceres::LossFunction* loss_function){
    colmap::Point3D& curr_3d = global_3d_map[point3D_id];

    // all obs already processed
    if (point3D_num_observations_[point3D_id] == curr_3d.Track().Length())
        return;

    for (const auto& track_el: curr_3d.Track().Elements()){
        // skip the obs if already added
        if (config_.HasImage(track_el.image_id)) {
            continue;
        }
        point3D_num_observations_[point3D_id] += 1;

        colmap::Image& track_image = global_image_map[track_el.image_id];
        const colmap::Point2D& track_point_2d = track_image.Point2D(track_el.point2D_idx);

        ceres::CostFunction* cost_function = nullptr;

        if (dataset_ == Dataset::Colmap){
            cost_function = BAConstPoseCostFxn<colmap::SimpleRadialCameraModel>::Create(track_image.Qvec(), track_image.Tvec(), track_point_2d.XY());                 
        }   
        else if (dataset_ == Dataset::Kitti){
            cost_function = BAConstPoseCostFxn<colmap::SimplePinholeCameraModel>::Create(track_image.Qvec(), track_image.Tvec(), track_point_2d.XY()); 
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

void BundleAdjust_::ParameterizeCameras(colmap::Camera& camera){
    const bool constant_camera = !options_.refine_focal_length &&
                                 !options_.refine_principal_point &&
                                 !options_.refine_extra_params;
    // we only have one camera, thus do not need to check constant cam id
    if (constant_camera) {
      problem_->SetParameterBlockConstant(camera.ParamsData());
    } else {
      std::vector<int> const_camera_params;

      if (!options_.refine_focal_length) {
        const std::vector<size_t>& params_idxs = camera.FocalLengthIdxs();
        const_camera_params.insert(const_camera_params.end(),
                                   params_idxs.begin(), params_idxs.end());
      }
      if (!options_.refine_principal_point) {
        const std::vector<size_t>& params_idxs = camera.PrincipalPointIdxs();
        const_camera_params.insert(const_camera_params.end(),
                                   params_idxs.begin(), params_idxs.end());
      }
      if (!options_.refine_extra_params) {
        const std::vector<size_t>& params_idxs = camera.ExtraParamsIdxs();
        const_camera_params.insert(const_camera_params.end(),
                                   params_idxs.begin(), params_idxs.end());
      }

      if (const_camera_params.size() > 0) {
        colmap::SetSubsetManifold(static_cast<int>(camera.NumParams()),
                          const_camera_params, problem_.get(),
                          camera.ParamsData());
      }
    }
}


