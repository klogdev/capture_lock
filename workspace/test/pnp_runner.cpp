#include <Eigen/Core>
#include <memory>

#include "test/pnp_test_template.h"
#include "test/pnp_test_data.h"
#include "test/pnp_runner.h"

#include "estimate/lhm.h"

#include "util_/math_helper.h"

void PnPTestRunner::run_test() {
    // Step 1: Generate data
    std::vector<std::vector<Eigen::Vector2d>> points2D;
    std::vector<Eigen::Vector3d> points3D;
    std::vector<Eigen::Matrix3x4d> gt_extrinsic;
    data_generator_->generate(points2D, points3D, gt_extrinsic);


    // Step 2: Estimate parameters using the generated data
    for(int i = 0; i < points3D.size(); i++) {
        // colmap's default implementation requires return a vector of models
        std::vector<Eigen::Matrix3x4d> estimated_extrinsic; 
        std::vector<double> residuals; // current residuals
        bool success = estimator_.estimate(points2D[i], points3D, 
                                            estimated_extrinsic, &residuals);

        // Check for estimation success
        if (!success) {
            std::cerr << "Estimation failed." << std::endl;
            return;
        }

        // Step 3: Evaluate and log the results
        // For demonstration, let's assume we're just printing the Frobenius norm
        // of the difference between the estimated and ground truth extrinsic matrices.
        double error = frobeniusNormExt(estimated_extrinsic[0], gt_extrinsic[i]);
        std::cout << "Estimation error (Frobenius norm): " << error << std::endl;

        // analyze and log residuals or other metrics
        double avg_residual = std::accumulate(residuals.begin(), residuals.end(), 0.0) / residuals.size();
        std::cout << "Average residual: " << avg_residual << std::endl;
    }
}

template<typename Estimator>
void convertData(const std::vector<Eigen::Vector2d>& points2D,
                 std::vector<typename Estimator::X_t>& converted_points2D) {
    converted_points2D.clear(); // Ensure the output vector is empty before starting
    converted_points2D.reserve(points2D.size()); // Reserve memory to avoid multiple allocations
    
    for (const auto& point : points2D) {
        // Assuming typename Estimator::X_t can be constructed from two doubles (x, y coordinates)
        converted_points2D.push_back(typename Estimator::X_t(point.x(), point.y()));
    }
}

template<typename Estimator>
void convertData(const std::vector<Eigen::Vector3d>& points3D,
                 std::vector<typename Estimator::Y_t>& converted_points3D) {
    converted_points3D.clear(); // Ensure the output vector is empty before starting
    converted_points3D.reserve(points3D.size()); // Reserve memory to avoid multiple allocations
    
    for (const auto& point : points3D) {
        // Assuming typename Estimator::X_t can be constructed from two doubles (x, y coordinates)
        converted_points3D.push_back(typename Estimator::Y_t(point.x(), point.y(), point.z()));
    }
}

template<typename Estimator>
void convertData(const std::vector<Eigen::Matrix3x4d>& extrinsic,
                 std::vector<typename Estimator::M_t>& converted_extrinsic) {
    converted_extrinsic.clear();
    converted_extrinsic.reserve(extrinsic.size());

    for (const auto& mat : extrinsic) {
        // Assuming typename Estimator::M_t can be directly assigned from Eigen::Matrix3x4d
        converted_extrinsic.push_back(typename Estimator::M_t(mat));
    }
}