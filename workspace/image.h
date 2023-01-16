#ifndef SRC_IMAGE_H_
#define SRC_IMAGE_H_

#include <string>
#include <vector>
#include <Eigen/Core>

#include "camera.h"
#include "types.h"
#include "point2d.h"

class Image {
    public:
    
    Image();

    // Setup / tear down the image and necessary internal data structures before
    // and after being used in reconstruction.
    void SetUp(const Camera& camera);

    image_t ImageId() const;
    inline std::string& Name();
    void SetImageId(const image_t image_id);
    void SetName(const std::string& name);

    camera_t CameraId() const;
    void SetCameraId(const camera_t camera_id);
    // Check whether identifier of camera has been set.
    bool HasCamera() const;

    // Check if image is registered.
    bool IsRegistered() const;
    void SetRegistered(const bool registered);

    // Get the number of image points.
    inline point2D_t NumPoints2D() const;

    // Get the number of triangulations, i.e. the number of points that
    // are part of a 3D point track.
    inline point2D_t NumPoints3D() const;

     // Access quaternion vector as (qw, qx, qy, qz) specifying the rotation of the
     // pose which is defined as the transformation from world to image space.
      inline const Eigen::Vector4d& Qvec() const;
      inline Eigen::Vector4d& Qvec();
      inline double Qvec(const size_t idx) const;
      inline double& Qvec(const size_t idx);
      inline void SetQvec(const Eigen::Vector4d& qvec);

    // Access the coordinates of image points.
    // need wrap the class of point2D&3D OR use a vector
    inline const class Point2D& Point2D(const point2D_t point2D_idx) const;
    inline class Point2D& Point2D(const point2D_t point2D_idx);
    inline const std::vector<class Point2D>& Points2D() const;
    void SetPoints2D(const std::vector<Eigen::Vector2d>& points);
    void SetPoints2D(const std::vector<class Point2D>& points);

    // Access the coordinates of image points. ATTENTION to s for Points
    inline Point2D& Point2D(const point2D_t point2D_idx) const;
    inline std::vector<class Point2D>& Points2D() const;

    ///
    //need setpoint3d, point2D vector call setpoint3d method
    ///

    // Set the point as triangulated, i.e. it is part of a 3D point track.
    void SetPoint3DForPoint2D(const uint32_t point2D_idx,
                              const uint32_t point3D_id);

    // Check whether one of the image points is part of the 3D point track.
    bool HasPoint3D(const point3D_t point3D_id) const;

    // Access translation vector as (tx, ty, tz) specifying the translation of the
    // pose which is defined as the transformation from world to image space.
    inline const Eigen::Vector3d& Tvec() const;
    inline Eigen::Vector3d& Tvec();
    inline double Tvec(const size_t idx) const;
    inline double& Tvec(const size_t idx);
    inline void SetTvec(const Eigen::Vector3d& tvec);

    // Convert Quaternion representation to 3D rotation matrix.
    //
    // @param qvec           Unit Quaternion rotation coefficients (w, x, y, z).
    //
    // @return               3x3 rotation matrix.
    Eigen::Matrix3d QuaternionToRotationMatrix(const Eigen::Vector4d& qvec);

    // Compose projection matrix from rotation and translation components.
    //
    // The projection matrix transforms 3D world to image points.
    //
    // @param qvec           Unit Quaternion rotation coefficients (w, x, y, z).
    // @param tvec           3x1 translation vector.
    //
    // @return               3x4 projection matrix.
    Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Vector4d& qvec,
                                              const Eigen::Vector3d& tvec);

    // Compose projection matrix from rotation matrix and translation components).
    //
    // The projection matrix transforms 3D world to image points.
    //
    // @param R              3x3 rotation matrix.
    // @param t              3x1 translation vector.
    //
    // @return               3x4 projection matrix.
    Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Matrix3d& R,
                                              const Eigen::Vector3d& T);

    // Compose the projection matrix from world to image space.
    Eigen::Matrix3x4d ProjectionMatrix() const;
    
    private:
    // Identifier of the image, if not specified `kInvalidImageId`.
    uint32_t image_id_;

    // The name of the image, i.e. the relative path.
    std::string name_;

    // The identifier of the associated camera. Note that multiple images might
    // share the same camera. If not specified `kInvalidCameraId`.
    uint32_t camera_id_;

    // Whether the image is successfully registered in the reconstruction.
    bool registered_;

    // The number of 3D points the image observes, i.e. the sum of its `points2D`
    // where `point3D_id != kInvalidPoint3DId`.
    uint32_t num_points3D_;

    // The number of image points that have at least one correspondence to
    // another image.
    uint32_t num_observations_;

    // The sum of correspondences per image point.
    uint32_t num_correspondences_;

    // The number of 2D points, which have at least one corresponding 2D point in
    // another image that is part of a 3D point track, i.e. the sum of `points2D`
    // where `num_tris > 0`.
    uint32_t num_visible_points3D_;

    // The pose of the image, defined as the transformation from world to image.
    Eigen::Vector4d qvec_;
    Eigen::Vector3d tvec_;

    // The pose prior of the image, e.g. extracted from EXIF tags.
    Eigen::Vector4d qvec_prior_;
    Eigen::Vector3d tvec_prior_;

    // All image points, including points that are not part of a 3D point track.
    std::vector<class Point2D> points2D_;

    // Per image point, the number of correspondences that have a 3D point.
    std::vector<uint32_t> num_correspondences_have_point3D_;
    };

    ////////////////////////////////////////////////////////////////////////////////
    // Implementation
    ////////////////////////////////////////////////////////////////////////////////

    image_t Image::ImageId() const { return image_id_; }

    void Image::SetImageId(const image_t image_id) { image_id_ = image_id; }

    std::string& Image::Name() { return name_; }

    void Image::SetName(const std::string& name) { name_ = name; }

    inline camera_t Image::CameraId() const { return camera_id_; }

    inline void Image::SetCameraId(const camera_t camera_id) {
      CHECK_NE(camera_id, kInvalidCameraId);
      camera_id_ = camera_id;
    }

    inline bool Image::HasCamera() const { return camera_id_ != kInvalidCameraId; }

    bool Image::IsRegistered() const { return registered_; }

    void Image::SetRegistered(const bool registered) { registered_ = registered; }

    class Point2D& Image::Point2D(const point2D_t point2D_idx) {
      return points2D_.at(point2D_idx);
    }

    const std::vector<class Point2D>& Image::Points2D() const { return points2D_; }
    #endif //SRC_IMAGE_H_