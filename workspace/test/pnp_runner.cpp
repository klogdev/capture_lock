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
    std::vector<std::vector<Eigen::Vector3d>> points3D;
    std::vector<Eigen::Matrix3x4d> gt_extrinsic;
    data_generator_->generate(points2D, points3D, gt_extrinsic);

    // Step 2: Estimate parameters using the generated data
    for(int i = 0; i < gt_extrinsic.size(); i++) {
        // colmap's default implementation requires return a vector of models
        std::vector<Eigen::Matrix3x4d> estimated_extrinsic; 
        std::vector<double> residuals; // current residuals
        std::cout << "DEBUGGING: points' size" << std::endl;
        std::cout << "curr 2d: " << points2D[i].size() << std::endl;
        std::cout << "3d: " << points3D[i].size() << std::endl;

        bool success = estimator_.estimate(points2D[i], points3D[i], 
                                           estimated_extrinsic, &residuals);
        
        Eigen::Matrix3x4d manual_extrinsic; 
        LHMEstimator lhm;
        bool manual_lhm = lhm.ComputeLHMPose(points2D[i], points3D[i], 
                                         &manual_extrinsic);
        
        std::cout << "current g.t. pose is: " << std::endl;
        std::cout << gt_extrinsic[i] << std::endl;

        std::cout << "manually estimated lhm is: " << std::endl;
        std::cout << manual_extrinsic << std::endl;
        
        // Check for estimation success
        if (!success) {
            std::cerr << "Estimation failed." << std::endl;
            return;
        }
        std::cout << "current estimated pose is: " << std::endl;
        std::cout << estimated_extrinsic[0] << std::endl;
        std::cout << "size of models is: " << estimated_extrinsic.size() << std::endl;

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
