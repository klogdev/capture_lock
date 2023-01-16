#ifndef SRC_POINT3D_H_
#define SRC_POINT3D_H_

#include <vector>

#include <Eigen/Core>
class Point3D {
 public:

  Point3D();

  // The point coordinate in world space.
  inline const Eigen::Vector3d& XYZ() const;
  inline double XYZ(const size_t idx) const;
  inline double X() const;
  inline double Y() const;
  inline double Z() const;
  inline void SetXYZ(const Eigen::Vector3d& xyz);

  private:
  // The 3D position of the point.
  Eigen::Vector3d xyz_;

};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

const Eigen::Vector3d& Point3D::XYZ() const { return xyz_; }

double Point3D::XYZ(const size_t idx) const { return xyz_(idx); }

double Point3D::X() const { return xyz_.x(); }

double Point3D::Y() const { return xyz_.y(); }

double Point3D::Z() const { return xyz_.z(); }

void Point3D::SetXYZ(const Eigen::Vector3d& xyz) { xyz_ = xyz; }


#endif  // SRC_POINT3D_H_
