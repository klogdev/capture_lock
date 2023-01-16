#ifndef SRC_CAMERA_MODELS_H_
#define SRC_CAMERA_MODELS_H_

#include <string>
#include <vector>
#include <unordered_map>


static const int InvalidCameraModelId = -1;

//macro that initialize different camera models
//include accesor of initialized camera
#ifndef CAMERA_MODEL_DEFINITIONS
#define CAMERA_MODEL_DEFINITIONS(model_id_value, model_name_value,             \
                                 num_params_value)                             \
  static const int kModelId = model_id_value;                                  \
  static const size_t kNumParams = num_params_value;                           \
  static const int model_id;                                                   \
  static const std::string model_name;                                         \
  static const size_t num_params;                                              \
  static const std::string params_info;                                        \
  static const std::vector<size_t> focal_length_idxs;                          \
  static const std::vector<size_t> principal_point_idxs;                       \
  static const std::vector<size_t> extra_params_idxs;                          \
  
  static inline int InitializeModelId() { return model_id_value; };            \
  static inline std::string InitializeModelName() {                            \
    return model_name_value;                                                   \
  };                                                                           \
  static inline size_t InitializeNumParams() { return num_params_value; };     \
  static inline std::string InitializeParamsInfo();                            \
  static inline std::vector<size_t> InitializeFocalLengthIdxs();               \
  static inline std::vector<size_t> InitializePrincipalPointIdxs();            \
  static inline std::vector<size_t> InitializeExtraParamsIdxs();               \
  static inline std::vector<double> InitializeParams(                          \
      const double focal_length, const size_t width, const size_t height);     \
                                                                               \
  template <typename T>                                                        \
  static void WorldToImage(const T* params, const T u, const T v, T* x, T* y); \
  template <typename T>                                                        \
  static void ImageToWorld(const T* params, const T x, const T y, T* u, T* v); \
  template <typename T>                                                        \
  static void Distortion(const T* extra_params, const T u, const T v, T* du,   \
                         T* dv);
#endif

#ifndef CAMERA_MODEL_CASES
#define CAMERA_MODEL_CASES                          \
  CAMERA_MODEL_CASE(SimplePinholeCameraModel)       \
  CAMERA_MODEL_CASE(PinholeCameraModel)             
  
#endif

#ifndef CAMERA_MODEL_SWITCH_CASES
#define CAMERA_MODEL_SWITCH_CASES         \
  CAMERA_MODEL_CASES                      \
  default:                                \
    CAMERA_MODEL_DOES_NOT_EXIST_EXCEPTION \
    break;
#endif

#define CAMERA_MODEL_DOES_NOT_EXIST_EXCEPTION \
  throw std::domain_error("Camera model does not exist")

// The "Curiously Recurring Template Pattern" (CRTP) is used here, so that we
// can reuse some shared functionality between all camera models -
// defined in the BaseCameraModel.
//Standard Template to define cameras
template <typename CameraModel>
struct BaseCameraModel {
  template <typename T>
  static inline bool HasBogusParams(const std::vector<T>& params,
                                    const size_t width, const size_t height,
                                    const T min_focal_length_ratio,
                                    const T max_focal_length_ratio,
                                    const T max_extra_param);

  template <typename T>
  static inline bool HasBogusFocalLength(const std::vector<T>& params,
                                         const size_t width,
                                         const size_t height,
                                         const T min_focal_length_ratio,
                                         const T max_focal_length_ratio);

  template <typename T>
  static inline bool HasBogusPrincipalPoint(const std::vector<T>& params,
                                            const size_t width,
                                            const size_t height);

  template <typename T>
  static inline bool HasBogusExtraParams(const std::vector<T>& params,
                                         const T max_extra_param);

  template <typename T>
  static inline T ImageToWorldThreshold(const T* params, const T threshold);

  template <typename T>
  static inline void IterativeUndistortion(const T* params, T* u, T* v);
};

// Simple Pinhole camera model.
//
// No Distortion is assumed. Only focal length and principal point is modeled.
//
// Parameter list is expected in the following order:
//
//   f, cx, cy
//
// See https://en.wikipedia.org/wiki/Pinhole_camera_model
struct SimplePinholeCameraModel
    : public BaseCameraModel<SimplePinholeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(0, "SIMPLE_PINHOLE", 3) //initialize a camera by macro
};

// Pinhole camera model.
//
// No Distortion is assumed. Only focal length and principal point is modeled.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy
//
// See https://en.wikipedia.org/wiki/Pinhole_camera_model
struct PinholeCameraModel : public BaseCameraModel<PinholeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(1, "PINHOLE", 4)
};

// Check whether camera model with given name or identifier exists.
bool ExistsCameraModelWithName(const std::string& model_name);
bool ExistsCameraModelWithId(const int model_id);

// Convert camera name to unique camera model identifier.
//
// @param name         Unique name of camera model.
//
// @return             Unique identifier of camera model.
int CameraModelNameToId(const std::string& model_name);

// Convert camera model identifier to unique camera model name.
//
// @param model_id     Unique identifier of camera model.
//
// @return             Unique name of camera model.
std::string CameraModelIdToName(const int model_id);

// Initialize camera parameters using given image properties.
//
// Initializes all focal length parameters to the same given focal length and
// sets the principal point to the image center.
//
// @param model_id      Unique identifier of camera model.
// @param focal_length  Focal length, equal for all focal length parameters.
// @param width         Sensor width of the camera.
// @param height        Sensor height of the camera.
std::vector<double> CameraModelInitializeParams(const int model_id,
                                                const double focal_length,
                                                const size_t width,
                                                const size_t height);

// Get human-readable information about the parameter vector order.
//
// @param model_id     Unique identifier of camera model.
std::string CameraModelParamsInfo(const int model_id);

// Get the indices of the parameter groups in the parameter vector.
//
// @param model_id     Unique identifier of camera model.
const std::vector<size_t>& CameraModelFocalLengthIdxs(const int model_id);
const std::vector<size_t>& CameraModelPrincipalPointIdxs(const int model_id);
const std::vector<size_t>& CameraModelExtraParamsIdxs(const int model_id);

// Get the total number of parameters of a camera model.
size_t CameraModelNumParams(const int model_id);

// Transform world coordinates in camera coordinate system to image coordinates.
//
// This is the inverse of `CameraModelImageToWorld`.
//
// @param model_id     Unique model_id of camera model as defined in
//                     `CAMERA_MODEL_NAME_TO_CODE`.
// @param params       Array of camera parameters.
// @param u, v         Coordinates in camera system as (u, v, 1).
// @param x, y         Output image coordinates in pixels.
inline void CameraModelWorldToImage(const int model_id,
                                    const std::vector<double>& params,
                                    const double u, const double v, double* x,
                                    double* y);

// Transform image coordinates to world coordinates in camera coordinate system.
//
// This is the inverse of `CameraModelWorldToImage`.
//
// @param model_id      Unique identifier of camera model.
// @param params        Array of camera parameters.
// @param x, y          Image coordinates in pixels.
// @param v, u          Output Coordinates in camera system as (u, v, 1).
inline void CameraModelImageToWorld(const int model_id,
                                    const std::vector<double>& params,
                                    const double x, const double y, double* u,
                                    double* v);

////////////////////////////////////////////////////////////////////////////////
// Implementation & Accessors
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// BaseCameraModel

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusParams(
    const std::vector<T>& params, const size_t width, const size_t height,
    const T min_focal_length_ratio, const T max_focal_length_ratio,
    const T max_extra_param) {
  if (HasBogusPrincipalPoint(params, width, height)) {
    return true;
  }

  if (HasBogusFocalLength(params, width, height, min_focal_length_ratio,
                          max_focal_length_ratio)) {
    return true;
  }

  if (HasBogusExtraParams(params, max_extra_param)) {
    return true;
  }

  return false;
}

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusFocalLength(
    const std::vector<T>& params, const size_t width, const size_t height,
    const T min_focal_length_ratio, const T max_focal_length_ratio) {
  const size_t max_size = std::max(width, height);

  for (const auto& idx : CameraModel::focal_length_idxs) {
    const T focal_length_ratio = params[idx] / max_size;
    if (focal_length_ratio < min_focal_length_ratio ||
        focal_length_ratio > max_focal_length_ratio) {
      return true;
    }
  }

  return false;
}

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusPrincipalPoint(
    const std::vector<T>& params, const size_t width, const size_t height) {
  const T cx = params[CameraModel::principal_point_idxs[0]];
  const T cy = params[CameraModel::principal_point_idxs[1]];
  return cx < 0 || cx > width || cy < 0 || cy > height;
}

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusExtraParams(
    const std::vector<T>& params, const T max_extra_param) {
  for (const auto& idx : CameraModel::extra_params_idxs) {
    if (std::abs(params[idx]) > max_extra_param) {
      return true;
    }
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
// SimplePinholeCameraModel

//print params name as a string
std::string SimplePinholeCameraModel::InitializeParamsInfo() {
  return "f, cx, cy";
}
//sepcify idx of focal
std::vector<size_t> SimplePinholeCameraModel::InitializeFocalLengthIdxs() {
  return {0};
}
//specify idx of principal
std::vector<size_t> SimplePinholeCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t> SimplePinholeCameraModel::InitializeExtraParamsIdxs() {
  return {};
}

std::vector<double> SimplePinholeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0};//why divide by 2.0??
}

template <typename T>
void SimplePinholeCameraModel::WorldToImage(const T* params, const T u,
                                            const T v, T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // No Distortion

  // Transform to image coordinates
  *x = f * u + c1; //world coord only 2 dims?
  *y = f * v + c2;
}

template <typename T>
void SimplePinholeCameraModel::ImageToWorld(const T* params, const T x,
                                            const T y, T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  *u = (x - c1) / f;
  *v = (y - c2) / f;
}

////////////////////////////////////////////////////////////////////////////////
// PinholeCameraModel

std::string PinholeCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy";
}

std::vector<size_t> PinholeCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> PinholeCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> PinholeCameraModel::InitializeExtraParamsIdxs() {
  return {};
}

std::vector<double> PinholeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0};
}

template <typename T>
void PinholeCameraModel::WorldToImage(const T* params, const T u, const T v,
                                      T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // No Distortion

  // Transform to image coordinates
  *x = f1 * u + c1;
  *y = f2 * v + c2;
}

template <typename T>
void PinholeCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                      T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  *u = (x - c1) / f1;
  *v = (y - c2) / f2;
}

/////////
void CameraModelWorldToImage(const int model_id,
                             const std::vector<double>& params, const double u,
                             const double v, double* x, double* y) {
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                    \
  case CameraModel::kModelId:                             \
    CameraModel::WorldToImage(params.data(), u, v, x, y); \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }
}

void CameraModelImageToWorld(const int model_id,
                             const std::vector<double>& params, const double x,
                             const double y, double* u, double* v) {
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                    \
  case CameraModel::kModelId:                             \
    CameraModel::ImageToWorld(params.data(), x, y, u, v); \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }
}


#endif  // SRC_CAMERA_MODELS_H_