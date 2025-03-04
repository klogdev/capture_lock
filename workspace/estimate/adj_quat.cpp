#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include <iostream>
#include "util_/math_helper.h"

void MakeDeterminants2D(const std::vector<Eigen::Vector3d>& points3D, 
                        const std::vector<Eigen::Vector2d>& points2D, 
                        std::vector<double>& dets,
                        double& eigenvalue_ratio) {
    // Ensure dets is empty or has the right size
    dets.clear();
    dets.reserve(10);

    // Extract x, y, and z components from points3D
    Eigen::ArrayXd x_components(points3D.size());
    Eigen::ArrayXd y_components(points3D.size());
    Eigen::ArrayXd z_components(points3D.size());

    for (size_t i = 0; i < points3D.size(); i++) {
        x_components(i) = points3D[i].x();
        y_components(i) = points3D[i].y();
        z_components(i) = points3D[i].z();
    }

    Eigen::ArrayXd u_components(points2D.size());
    Eigen::ArrayXd v_components(points2D.size());
    // Extract u and v components from points2D
    for (size_t i = 0; i < points2D.size(); i++) {
        u_components(i) = points2D[i].x();
        v_components(i) = points2D[i].y();
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

    // Check eigenvalue ratio of mat1
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(mat1);
    if (eigensolver.info() == Eigen::Success) {
        double smallest_eigenvalue = eigensolver.eigenvalues()(0); // eigenvalues are sorted in ascending order
        double largest_eigenvalue = eigensolver.eigenvalues()(2);
        eigenvalue_ratio = std::abs(smallest_eigenvalue / largest_eigenvalue);
    }
}

void MakeRTilde(const std::vector<Eigen::Vector3d>& points3D, 
                const std::vector<Eigen::Vector2d>& points2D,
                Eigen::Matrix3d& r_tilde) {
    std::vector<double> dets;
    double eigenvalue_ratio;

    MakeDeterminants2D(points3D, points2D, dets, eigenvalue_ratio);

    r_tilde << dets[6] / dets[0], -dets[3] / dets[0], dets[1] / dets[0],
               dets[7] / dets[0], -dets[4] / dets[0], dets[2] / dets[0],
               dets[5] / dets[0],  dets[8] / dets[0], dets[9] / dets[0];
}

void MMatrix(const std::vector<Eigen::Vector3d>& points3D, 
             const std::vector<Eigen::Vector2d>& points2D,
             Eigen::Matrix4d& M, double& eigenvalue_ratio) {
    std::vector<double> dets; // this BI correction still need DRaM, but not for R tilde
    MakeDeterminants2D(points3D, points2D, dets, eigenvalue_ratio);

    M << dets[6] - dets[4] + dets[9], -(dets[2] - dets[8]), -(dets[5] - dets[1]), -(-dets[3] - dets[7]),
        -(dets[2] - dets[8]), dets[6] + dets[4] - dets[9], -dets[3] + dets[7], dets[5] + dets[1],
        -(dets[5] - dets[1]), -dets[3] + dets[7], -dets[6] - dets[4] - dets[9], dets[2] + dets[8],
        -(-dets[3] - dets[7]), dets[5] + dets[1], dets[2] + dets[8], -dets[6] + dets[4] + dets[9];

    // Ensure d1 is not zero to avoid division by zero
    // if (std::abs(dets[0]) > std::numeric_limits<double>::epsilon()) {
    //     M /= dets[0];
    // } else {
    //     // Handle the zero division case, possibly set M to zero or an identity matrix, or handle as per your application's needs
    //     std::cerr << "cannot divided by zero when constructing M" << std::endl;
    // }
}

void MakeAdjugate(Eigen::Matrix4d& matrix, Eigen::Matrix4d& adjugate) {
    Eigen::Matrix4d cofactorMatrix;

    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            // Create the minor matrix by removing the current row and column
            Eigen::Matrix3d minor; // init the minor matrix
            int minor_row = 0;
            for (int i = 0; i < 4; ++i) {
                if (i == row) continue;
                int minor_col = 0;
                for (int j = 0; j < 4; ++j) {
                    if (j == col) continue;
                    minor(minor_row, minor_col) = matrix(i, j);
                    minor_col++;
                }
                minor_row++;
            }

            // Compute the cofactor
            double cofactor = std::pow(-1, row + col) * minor.determinant();
            cofactorMatrix(row, col) = cofactor;
        }
    }

    // The adjugate is the transpose of the cofactor matrix
    adjugate = cofactorMatrix.transpose();
}

bool BarItzhackOptRot(const std::vector<Eigen::Vector3d>& points3D, 
                      const std::vector<Eigen::Vector2d>& points2D,
                      Eigen::Matrix3d& opt_rot) {
    Eigen::Matrix4d M;
    double eigenvalue_ratio;
    MMatrix(points3D, points2D, M, eigenvalue_ratio);

    if (eigenvalue_ratio < 2.4e-2) {
        std::cerr << "Warning: Planar case detected. Eigenvalue ratio: " << eigenvalue_ratio << std::endl;
        return false;
    }

    // Eigenvalue decomposition of M
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(M);
    if (eigensolver.info() != Eigen::Success) {
        // Handle the error - computation failed
        std::cerr << "Eigenvalue decomposition failed!" << std::endl;
        opt_rot = Eigen::Matrix3d::Identity(); 
        return false;
    }

    Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
    Eigen::Matrix4d chi = M - Eigen::Matrix4d::Identity() * eigenvalues(3);

    Eigen::Matrix4d adjugate; 
    MakeAdjugate(chi, adjugate);

    // Finding the quaternion with the maximum diagonal element
    int max_diag_index = 0;
    double max_diag_value = -1.0;
    for (int i = 0; i < 4; ++i) {
        double diag_value = std::abs(adjugate(i, i));
        if (diag_value > max_diag_value) {
            max_diag_value = diag_value;
            max_diag_index = i;
        }
    }

    Eigen::Vector4d quat_estimate = adjugate.row(max_diag_index).normalized();

    // Converting quaternion to rotation matrix
    Eigen::Quaterniond quat(quat_estimate(0), quat_estimate(1), quat_estimate(2), quat_estimate(3));
    opt_rot = quat.toRotationMatrix();

    return true;
}


