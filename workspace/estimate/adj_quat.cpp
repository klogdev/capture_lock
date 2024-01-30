#include <Eigen/Dense>
#include <vector>

void MakeDeterminants(const std::vector<Eigen::Vector3d>& points3D, 
                      const std::vector<Eigen::Vector2d>& points2D, 
                      std::vector<double>& dets) {
    // Ensure dets is empty or has the right size
    dets.clear();
    dets.reserve(10);

    double XX = points3D[0].dot(points3D[0]);
    double XY = points3D[0].dot(points3D[1]);
    double XZ = points3D[0].dot(points3D[2]);
    double YY = points3D[1].dot(points3D[1]);
    double YZ = points3D[1].dot(points3D[2]);
    double ZZ = points3D[2].dot(points3D[2]);
    double UX = points2D[0].dot(points3D[0].head<2>());
    double UY = points2D[0].dot(points3D[1].head<2>());
    double UZ = points2D[0].dot(points3D[2].head<2>());
    double VX = points2D[1].dot(points3D[0].head<2>());
    double VY = points2D[1].dot(points3D[1].head<2>());
    double VZ = points2D[1].dot(points3D[2].head<2>());

    Eigen::Matrix3d mat1, mat2, mat3, mat4, mat5, mat6, mat7, mat8, mat9, mat10;
    // Matrix initialization with XX, XY, XZ, YY, YZ, ZZ, UX, UY, UZ, VX, VY, VZ
    mat1 << XX, XY, XZ, XY, YY, YZ, XZ, YZ, ZZ;
    mat2 << XX, XY, UX, XY, YY, UY, XZ, YZ, UZ;
    mat3 << XX, XY, VX, XY, YY, VY, XZ, YZ, VZ;
    mat4 << XX, XZ, UX, XY, YZ, UY, XZ, ZZ, UZ;
    mat5 << XX, XZ, VX, XY, YZ, VY, XZ, ZZ, VZ;
    mat6 << XX, UX, VX, XY, UY, VY, XZ, UZ, VZ;
    mat7 << XY, XZ, UX, YY, YZ, UY, YZ, ZZ, UZ;
    mat8 << XY, XZ, VX, YY, YZ, VY, YZ, ZZ, VZ;
    mat9 << XY, UX, VX, YY, UY, VY, YZ, UZ, VZ;
    mat10 << XZ, UX, VX, YZ, UY, VY, ZZ, UZ, VZ;

    // Store the determinants in the vector
    dets.push_back(mat1.determinant());
    dets.push_back(mat2.determinant());
    dets.push_back(mat3.determinant());
    dets.push_back(mat4.determinant());
    dets.push_back(mat5.determinant());
    dets.push_back(mat6.determinant());
    dets.push_back(mat7.determinant());
    dets.push_back(mat8.determinant());
    dets.push_back(mat9.determinant());
    dets.push_back(mat10.determinant());
}

void MakeRTilde(const std::vector<Eigen::Vector3d>& points3D, 
                const std::vector<Eigen::Vector2d>& points2D,
                Eigen::Matrix3d& r_tilde); {
    std::vector<double> dets;
    MakeDeterminants(points3D, points2D, dets);

    r_tilde << dets[6] / dets[0], -dets[3] / dets[0], dets[1] / dets[0],
               dets[7] / dets[0], -dets[4] / dets[0], dets[2] / dets[0],
               dets[5] / dets[0],  dets[8] / dets[0], dets[9] / dets[0];
}