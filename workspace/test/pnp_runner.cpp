#include <Eigen/Core>
#include <memory>

#include "test/pnp_test_template.h"
#include "test/pnp_test_data.h"

#include "util_/math.h"

void PnPTestRunner::run_test() {
    // Step 1: Generate data
    std::vector<std::vector<Eigen::Vector2d>> points2D;
    std::vector<Eigen::Vector3d> points3D;
    std::vector<Eigen::Matrix3x4d> gt_extrinsic;
    dataGenerator_->generate(points2D, points3D, gt_extrinsic);

    // Step 2: Estimate parameters using the generated data
    for(int i = 0; i < points3D.size(); i++) {
        Eigen::Matrix3x4d estimated_extrinsic;
        std::vector<double> residuals;
        bool success = estimator_.estimate(points2D[i], points3D, estimated_extrinsic, &residuals);

        // Check for estimation success
        if (!success) {
            std::cerr << "Estimation failed." << std::endl;
            return;
        }

        // Step 3: Evaluate and log the results
        // For demonstration, let's assume we're just printing the Frobenius norm
        // of the difference between the estimated and ground truth extrinsic matrices.
        double error = frobeniusNorm(estimated_extrinsic, gt_extrinsic[i]);
        std::cout << "Estimation error (Frobenius norm): " << error << std::endl;

        // analyze and log residuals or other metrics
        double avg_residual = std::accumulate(residuals.begin(), residuals.end(), 0.0) / residuals.size();
        std::cout << "Average residual: " << avg_residual << std::endl;
    }
}
