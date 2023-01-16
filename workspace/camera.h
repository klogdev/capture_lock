#ifndef SRC_CAMERA_H_
#define SRC_CAMERA_H_

#include <vector>
#include <cstdint>
#include "types.h"

class Camera{
    public:
    Camera();

    // Access the unique identifier of the camera.
    inline int CameraId() const;
    inline void SetCameraId(const int32_t camera_id);

    // Access the camera model.
    inline int ModelId() const;
    std::string ModelName() const;
    void SetModelId(const int model_id);
    void SetModelIdFromName(const std::string& model_name);

    // Access dimensions of the camera sensor.
    inline size_t Width() const;
    inline size_t Height() const;
    inline void SetWidth(const size_t width);
    inline void SetHeight(const size_t height);
    inline const std::vector<double>& Params();

    // Access focal length parameters.
    double MeanFocalLength() const;
    double FocalLength() const;
    double FocalLengthX() const;
    double FocalLengthY() const;
    void SetFocalLength(const double focal_length);
    void SetFocalLengthX(const double focal_length_x);
    void SetFocalLengthY(const double focal_length_y);

    // Check if camera has prior focal length.
    inline bool HasPriorFocalLength() const;
    inline void SetPriorFocalLength(const bool prior);

    // Access principal point parameters. Only works if there are two
    // principal point parameters.
    double PrincipalPointX() const;
    double PrincipalPointY() const;
    void SetPrincipalPointX(const double ppx);
    void SetPrincipalPointY(const double ppy);

    // Get the indices of the parameter groups in the parameter vector.
    const std::vector<size_t>& FocalLengthIdxs() const;
    const std::vector<size_t>& PrincipalPointIdxs() const;
    const std::vector<size_t>& ExtraParamsIdxs() const;

    // Get intrinsic calibration matrix composed from focal length and principal
    // point parameters, excluding distortion parameters.
    Eigen::Matrix3d CalibrationMatrix() const;

    // Get human-readable information about the parameter vector ordering.
    std::string ParamsInfo() const;

    private:
    // The unique identifier of the camera. If the identifier is not specified
    // it is set to `kInvalidCameraId`.
    int32_t camera_id_;

    // The identifier of the camera model. If the camera model is not specified
    // the identifier is `kInvalidCameraModelId`.
    int model_id_;

    // The dimensions of the image, 0 if not initialized.
    size_t width_;
    size_t height_;

    // The focal length, principal point, and extra parameters. If the camera
    // model is not specified, this vector is empty.
    std::vector<double> params_;

};
//////////////
//Accessors
/////////////
int Camera::CameraId() const { return camera_id_; }

void Camera::SetCameraId(const int32_t camera_id) { camera_id_ = camera_id; }

int Camera::ModelId() const { return model_id_; }

size_t Camera::Width() const { return width_; }

size_t Camera::Height() const { return height_; }

void Camera::SetWidth(const size_t width) { width_ = width; }

void Camera::SetHeight(const size_t height) { height_ = height; }

const std::vector<double>& Camera::Params() { return params_; }

#endif  // SRC_CAMERA_H_
