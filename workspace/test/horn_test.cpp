#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <limits>
#include <iostream>

#include "test/horn_test.h"

bool HornRot(std::vector<Eigen::Vector3d>& points3D0,
            std::vector<Eigen::Vector3d>& points3D1,
                            Eigen::Matrix3d& R) {
    Eigen::Vector3d pc = Eigen::Vector3d::Zero();
    Eigen::Vector3d qc = Eigen::Vector3d::Zero();

    // here we use (u,v,1) for the points3D1
    GetCentroidUtil(points3D0, points3D1, pc, qc);

    // Compute M from centered points and dq, dp's squares
    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

    double dpsum = 0.0, dqsum = 0.0;

    for (size_t i = 0; i < points3D0.size(); i++) {
        Eigen::Vector3d p = points3D0[i] - pc;
        Eigen::Vector3d q = points3D1[i] - qc;

        dpsum += p.squaredNorm();
        dqsum += q.squaredNorm();

        S += p * q.transpose(); // transform p to q -> p*q.T
    }

    // Diagonal elements
    M(0, 0) = S.trace(); // Sxx + Syy + Szz
    M(1, 1) = S(0, 0) - S(1, 1) - S(2, 2); // Sxx - Syy - Szz
    M(2, 2) = -S(0, 0) + S(1, 1) - S(2, 2); // -Sxx + Syy - Szz
    M(3, 3) = -S(0, 0) - S(1, 1) + S(2, 2); // -Sxx - Syy + Szz

    // Off-diagonal elements
    M(0, 1) = M(1, 0) = S(1, 2) - S(2, 1); // Syz - Szy
    M(0, 2) = M(2, 0) = S(2, 0) - S(0, 2); // Szx - Sxz
    M(0, 3) = M(3, 0) = S(0, 1) - S(1, 0); // Sxy - Syx

    M(1, 2) = M(2, 1) = S(0, 1) + S(1, 0); // Sxy + Syx
    M(1, 3) = M(3, 1) = S(2, 0) + S(0, 2); // Szx + Sxz

    M(2, 3) = M(3, 2) = S(1, 2) + S(2, 1); // Syz + Szy

    double scale = std::sqrt(dqsum / dpsum);

    // Eigenvalue decomposition of a symmetric matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(M);
    if (eigensolver.info() != Eigen::Success) {
        std::cerr << "Eigenvalue decomposition failed!" << std::endl;
        return false;
    }

    // use the eigensolver
    Eigen::Vector4d eigenvec = eigensolver.eigenvectors().col(3); // Last column for largest eigenvalue

    // Convert Quaternion to Rotation Matrix
    // Normalize the eigenvector to ensure it represents a valid rotation
    eigenvec.normalize();

    // Create a Quaternion from the eigenvector
    Eigen::Quaterniond quaternion(eigenvec(0), eigenvec(1), eigenvec(2), eigenvec(3));
    
    // Convert the quaternion to a rotation matrix
    R = quaternion.toRotationMatrix();

    return true;
}

void GetCentroidUtil(std::vector<Eigen::Vector3d>& points3D0,
                    std::vector<Eigen::Vector3d>& points3D1,
                    Eigen::Vector3d& pc, Eigen::Vector3d& qc) {
    // Compute centroids
    for (const auto& p : points3D0) {
        pc += p;
    }
    for (const auto& q : points3D1) {
        qc += q;
    }
    pc /= points3D0.size();
    qc /= points3D1.size();
}

