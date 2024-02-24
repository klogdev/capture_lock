#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

/**
 * @brief here we make sure the Eigen uses Hamiltonian convention
*/
int main() {

    Eigen::Quaterniond q1(1.0, 0.0, 0.0, 0.0); // Identity quaternion
    Eigen::Quaterniond q2(0.0, 0.0, 0.0, 1.0); // Identity quaternion

    Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();

    const Eigen::Quaterniond quat(mat);
    std::cout << "vector convert from identity mat" << std::endl;
    std::cout << Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z()) << std::endl;


    std::cout << "the first quat's rot mat is: " << std::endl;
    std::cout << q1.toRotationMatrix() << std::endl;
    std::cout << "the second quat's rot mat is: " << std::endl;
    std::cout << q2.toRotationMatrix() << std::endl;

}