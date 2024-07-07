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
    std::vector<double> first_quat_data;
    std::vector<double> time_data;
    std::vector<int> iter_data;
    std::vector<double> rot_data;
    std::vector<double> trans_data;

    // time series of obj error/relative quaternion error on the fly
    std::vector<std::vector<double>> obj_err_series;
    std::vector<std::vector<double>> quat_err_series;

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

        // record time series of metrics during iterations
        if(lhm_type_) {
            std::vector<double> curr_rel_quat = LHMEstimator::obj_errs;
            std::vector<double> curr_obj_err = LHMEstimator::rel_quats;
            obj_err_series.push_back(curr_obj_err);
            quat_err_series.push_back(curr_rel_quat);
        }
        
        Eigen::Matrix3x4d manual_extrinsic; 
        LHMEstimator lhm;
        bool manual_lhm = lhm.ComputeLHMPose(points2D[i], points3D[i], 
                                         &manual_extrinsic);
        
        double first_quat = lhm.first_estimated_rela_quat;
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
        double error = frobeniusNorm(estimated_extrinsic[0], gt_extrinsic[i]);
        // std::cout << "Estimation error (Frobenius norm): " << error << std::endl;

        // analyze and log residuals or other metrics
        double avg_residual = std::accumulate(residuals.begin(), residuals.end(), 0.0) / residuals.size();
                
        // calculate relative error
        double quat_err = RelativeQuatErr(gt_extrinsic[i].block<3, 3>(0, 0), estimated_extrinsic[0].block<3, 3>(0, 0));
        double trans_err = RelativeTransErr(gt_extrinsic[i].col(3), estimated_extrinsic[0].col(3));
        // append all metrics
        residual_data.push_back(avg_residual);
        frobenius_data.push_back(error);
        first_quat_data.push_back(first_quat);
        time_data.push_back(seconds_pnp);
        iter_data.push_back(iters);
        rot_data.push_back(quat_err);
        trans_data.push_back(trans_err);
    }
    std::string curr_sigma = std::to_string(sigma_);
    // save data
    save1DVec(residual_data, output_path_ + "_residuals_" + "_.txt");
    save1DVec(frobenius_data, output_path_ + "_frobenius_" + "_.txt");
    save1DVec(first_quat_data, output_path_ + "_first_quat_" + "_.txt");
    save1DVec(time_data, output_path_ + "_durations_" + "_.txt");
    save1DVec(rot_data, output_path_ + "_rot_" + "_.txt");
    save1DVec(trans_data, output_path_ + "_trans_" + "_.txt");
    save1DVec(iter_data, output_path_ + "_iters_" + "_.txt");
    // list of time series of LHM-rleated methods
    if(obj_err_series.size() != 0)
        save2DdoubleVec(obj_err_series, output_path_ + "_obj_space_err_" + curr_sigma + "_.txt");
    if(quat_err_series.size() != 0)
        save2DdoubleVec(quat_err_series, output_path_ + "_rela_quat_err_" + curr_sigma + "_.txt");
}
