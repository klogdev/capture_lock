#include "camera.h"
#include "camera_model.h"

Camera::Camera()
    : camera_id_(kInvalidCameraId),
      model_id_(kInvalidCameraModelId),
      width_(0),
      height_(0){}

//methods are from camera_model
std::string Camera::ModelName() const { return CameraModelIdToName(model_id_); }

void Camera::SetModelId(const int model_id) {
  CHECK(ExistsCameraModelWithId(model_id));
  model_id_ = model_id;
  params_.resize(CameraModelNumParams(model_id_), 0); //rezie the params_ to number of params model required with 0
}

void Camera::SetModelIdFromName(const std::string& model_name) {
  CHECK(ExistsCameraModelWithName(model_name));
  model_id_ = CameraModelNameToId(model_name);
  params_.resize(CameraModelNumParams(model_id_), 0);
}

const std::vector<size_t>& Camera::FocalLengthIdxs() const {
  return CameraModelFocalLengthIdxs(model_id_);
}

const std::vector<size_t>& Camera::PrincipalPointIdxs() const {
  return CameraModelPrincipalPointIdxs(model_id_);
}

const std::vector<size_t>& Camera::ExtraParamsIdxs() const {
  return CameraModelExtraParamsIdxs(model_id_);
}

double Camera::PrincipalPointX() const {
  const std::vector<size_t>& idxs = PrincipalPointIdxs();
  CHECK_EQ(idxs.size(), 2);
  return params_[idxs[0]];
}

double Camera::PrincipalPointY() const {
  const std::vector<size_t>& idxs = PrincipalPointIdxs();
  CHECK_EQ(idxs.size(), 2);
  return params_[idxs[1]];
}

Eigen::Matrix3d Camera::CalibrationMatrix() const {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity(); //initialize an identity matrix

  const std::vector<size_t>& idxs = FocalLengthIdxs(); //get idxs of focal, from camera model
  if (idxs.size() == 1) {   //one focal
    K(0, 0) = params_[idxs[0]];
    K(1, 1) = params_[idxs[0]];
  } else if (idxs.size() == 2) { //fx, fy
    K(0, 0) = params_[idxs[0]];
    K(1, 1) = params_[idxs[1]];
  } else {
    std::cout
        << "Camera model must either have 1 or 2 focal length parameters." << std::endl;
  }

  K(0, 2) = PrincipalPointX(); 
  K(1, 2) = PrincipalPointY();

  return K;
}

//from camera_model, return a string to remind which camera info it has
std::string Camera::ParamsInfo() const {
  return CameraModelParamsInfo(model_id_);
}

double Camera::MeanFocalLength() const {
  const auto& focal_length_idxs = FocalLengthIdxs();
  double focal_length = 0;
  for (const auto idx : focal_length_idxs) {
    focal_length += params_[idx];
  }
  return focal_length / focal_length_idxs.size();
}

