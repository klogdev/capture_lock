#include <Eigen/Core>
#include <memory>
#include <chrono>

#include "test/pnp_test_template.h"
#include "test/pnp_test_data.h"
#include "test/pnp_runner.h"

#include "estimate/lhm.h"

#include "util_/math_helper.h"
#include "util_/file_save.h"

void PnPTestRunner::run_test() {
    // Step 1: Generate data
    std::vector<std::vector<Eigen::Vector2d>> points2D;
    std::vector<std::vector<Eigen::Vector3d>> points3D;
    std::vector<Eigen::Matrix3x4d> gt_extrinsic;
    data_generator_->generate(points2D, points3D, gt_extrinsic);

    // initialize vectors to save metrics
    std::vector<double> residual_data;
    std::vector<double> frobenius_data;
    std::vector<double> dram_frob_data;
    std::vector<double> time_data;
    std::vector<int> iter_data;
    std::vector<double> rot_data;
    std::vector<double> trans_data;
    // Step 2: Estimate parameters using the generated data
    for(int i = 0; i < gt_extrinsic.size(); i++) {
        // colmap's default implementation requires return a vector of models
        std::vector<Eigen::Matrix3x4d> estimated_extrinsic; 
        std::vector<double> residuals; // current residuals

        LHMEstimator::setGroundTruthPose(&gt_extrinsic[i]);
        auto before_pnp = std::chrono::system_clock::now();
        bool success = estimator_.estimate(points2D[i], points3D[i], 
                                           estimated_extrinsic, &residuals);
        auto after_pnp = std::chrono::system_clock::now();

        auto duration_pnp = after_pnp - before_pnp;
        // Convert the duration to seconds as a double
        double seconds_pnp = std::chrono::duration<double>(duration_pnp).count();
        
        Eigen::Matrix3x4d manual_extrinsic; 
        LHMEstimator lhm;
        bool manual_lhm = lhm.ComputeLHMPose(points2D[i], points3D[i], 
                                         &manual_extrinsic);
        
        double first_frob = lhm.first_estimated_frob;
        int iters = lhm.num_iterations;

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
        // std::cout << "Estimation error (Frobenius norm): " << error << std::endl;

        // analyze and log residuals or other metrics
        double avg_residual = std::accumulate(residuals.begin(), residuals.end(), 0.0) / residuals.size();
                
        // calculate relative error
        double quat_err = RelativeQuatErr(gt_extrinsic[i], estimated_extrinsic[0]);
        double trans_err = RelativeTransErr(gt_extrinsic[i], estimated_extrinsic[0]);
        // append all metrics
        residual_data.push_back(avg_residual);
        frobenius_data.push_back(error);
        dram_frob_data.push_back(first_frob);
        time_data.push_back(seconds_pnp);
        iter_data.push_back(iters);
        rot_data.push_back(quat_err);
        trans_data.push_back(trans_err);
    }
    std::string curr_sigma = std::to_string(sigma_);
    // save data
    save1DVec(residual_data, output_path_ + "_residuals_" + curr_sigma + "_.txt");
    save1DVec(frobenius_data, output_path_ + "_frobenius_" + curr_sigma + "_.txt");
    save1DVec(dram_frob_data, output_path_ + "_first_frob_" + curr_sigma + "_.txt");
    save1DVec(time_data, output_path_ + "_durations_" + curr_sigma + "_.txt");
    save1DVec(rot_data, output_path_ + "_rot_" + curr_sigma + "_.txt");
    save1DVec(trans_data, output_path_ + "_trans_" + curr_sigma + "_.txt");
    save1DVec(iter_data, output_path_ + "_iters_" + curr_sigma + "_.txt");
}
