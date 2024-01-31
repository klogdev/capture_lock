#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include <iostream>

void MakeDeterminants2D(const std::vector<Eigen::Vector3d>& points3D, 
                      const std::vector<Eigen::Vector2d>& points2D, 
                      std::vector<double>& dets) {
    // Ensure dets is empty or has the right size
    dets.clear();
    dets.reserve(10);

    // Extract x, y, and z components from points3D
    Eigen::ArrayXd x_components(points3D.size());
    Eigen::ArrayXd y_components(points3D.size());
    Eigen::ArrayXd z_components(points3D.size());

    for (size_t i = 0; i < points3D.size(); ++i) {
        x_components(i) = points3D[i].x();
        y_components(i) = points3D[i].y();
        z_components(i) = points3D[i].z();
    }

    Eigen::ArrayXd u_components(points2D.size());
    Eigen::ArrayXd v_components(points2D.size());
    // Extract u and v components from points2D
    std::vector<double> u_components, v_components;
    for (const auto& point : points2D) {
        u_components(i) = points3D[i].x();
        v_components(i) = points3D[i].y();
    }

    // Obtain cross matrix's entries
    double XX = x_components.square().sum();
    double YY = y_components.square().sum();
    double ZZ = z_components.square().sum();

    double XY = (x_components * y_components).sum();
    double XZ = (x_components * z_components).sum();
    double YZ = (y_components * z_components).sum();

    double UX = (u_components * x_components).sum();
    double UY = (u_components * y_components).sum();
    double UZ = (u_components * z_components).sum();

    double VX = (v_components * x_components).sum();
    double VY = (v_components * y_components).sum();
    double VZ = (v_components * z_components).sum();

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
    MakeDeterminants2D(points3D, points2D, dets);

    r_tilde << dets[6] / dets[0], -dets[3] / dets[0], dets[1] / dets[0],
               dets[7] / dets[0], -dets[4] / dets[0], dets[2] / dets[0],
               dets[5] / dets[0],  dets[8] / dets[0], dets[9] / dets[0];
}

void MMatrix(const std::vector<Eigen::Vector3d>& points3D, 
             const std::vector<Eigen::Vector2d>& points2D,
             Eigen::Matrix4d& M) {
    std::vector<double> dets;
    MakeDeterminants2D(points3D, points2D, dets);

    M << dets[6] - dets[4] + dets[9], -(dets[2] - dets[8]), -(dets[5] - dets[1]), -(-dets[3] - dets[7]),
        -(dets[2] - dets[8]), dets[6] + dets[4] - dets[9], -dets[3] + dets[7], dets[5] + dets[1],
        -(dets[5] - dets[1]), -dets[3] + dets[7], -dets[6] - dets[4] - dets[9], dets[2] + dets[8],
        -(-dets[3] - dets[7]), dets[5] + dets[1], dets[2] + dets[8], -dets[6] + dets[4] + dets[9];

    // Ensure d1 is not zero to avoid division by zero
    if (std::abs(dets[0]) > std::numeric_limits<double>::epsilon()) {
        M /= dets[0];
    } else {
        // Handle the zero division case, possibly set M to zero or an identity matrix, or handle as per your application's needs
        M.setZero(); // Example: Setting M to zero
        std::cerr << "cannot divided by zero when constructing M" << std::endl;
    }
}
