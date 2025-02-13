#include <Eigen/Core>
#include <memory>
#include <chrono>
#include <boost/format.hpp>

#include "base/projection.h"
#include "base/camera.h"
#include "base/pose.h"

#include "pnp/pnp_test_template.h"
#include "pnp/pnp_test_data.h"
#include "pnp/pnp_runner.h"

#include "estimate/lhm.h"

#include "util_/math_helper.h"
#include "util_/file_save.h"

#include "file_reader/tum_rgbd.h"

void PnPTestRunner::run_test() {
    // Step 1: Generate data
    std::vector<std::vector<Eigen::Vector2d>> points2D;
    std::vector<std::vector<Eigen::Vector3d>> points3D;
    std::vector<Eigen::Matrix3x4d> gt_extrinsic;

    data_generator_->generate(points2D, points3D, gt_extrinsic);

    // initialize vectors to save metrics
    std::vector<std::vector<double>> residual_data;
    std::vector<double> frobenius_data;
    std::vector<double> time_data;
    std::vector<int> iter_data;
    std::vector<double> rot_data;
    std::vector<double> trans_data;
    std::vector<double> cosine_diff_data;

    // time series of obj error/relative quaternion error on the fly
    std::vector<std::vector<double>> obj_err_series;
    std::vector<std::vector<double>> quat_err_series;
    std::vector<std::vector<double>> cos_diff_series;

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
            std::vector<double> curr_cos_diff = LHMEstimator::cos_diffs;
            LHMEstimator::clearObjErrs();
            LHMEstimator::clearRelQuats();
            LHMEstimator::clearCosDiff();
            obj_err_series.push_back(curr_obj_err);
            quat_err_series.push_back(curr_rel_quat);
            cos_diff_series.push_back(curr_cos_diff);
        }
        
        int iters = LHMEstimator::num_iterations;

        std::cout << "current g.t. pose is: " << std::endl;
        std::cout << gt_extrinsic[i] << std::endl;
        
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
                
        // calculate relative error
        Eigen::Vector4d est_quat = colmap::RotationMatrixToQuaternion(estimated_extrinsic[0].block<3, 3>(0, 0));
        Eigen::Vector4d gt_quat = colmap::RotationMatrixToQuaternion(gt_extrinsic[i].block<3, 3>(0, 0));

        std::cout << "final estimated rot in quat: " << std::endl;
        std::cout << est_quat << std::endl;
        std::cout << "g.t. rot in quat: " << std::endl;
        std::cout << gt_quat << std::endl;

        double quat_err = RelativeQuatErr(gt_quat, est_quat);
        double trans_err = RelativeTransErr(gt_extrinsic[i].col(3), estimated_extrinsic[0].col(3));
        double cosine_diff = CosineDifference(gt_quat, est_quat);
        // append all metrics
        residual_data.push_back(residuals); // 2D vector
        frobenius_data.push_back(error);
        time_data.push_back(seconds_pnp);
        iter_data.push_back(iters);
        rot_data.push_back(quat_err);
        trans_data.push_back(trans_err);
        cosine_diff_data.push_back(cosine_diff);
    }
    std::string curr_sigma = std::to_string(sigma_);
    // save data
    save1DVec(frobenius_data, output_path_ + "_frobenius_" + ".txt");
    save1DVec(time_data, output_path_ + "_durations_" + ".txt");
    save1DVec(rot_data, output_path_ + "_rot_" + ".txt");
    save1DVec(trans_data, output_path_ + "_trans_" + ".txt");
    save1DVec(cosine_diff_data, output_path_ + "_cos_diff_" + ".txt");
    save1DVec(iter_data, output_path_ + "_iters_" + ".txt");

    // save all reprojection errors,
    // for the run with RANSAC, we need to pick reasonable residuals with inlier masks
    save2DdoubleVec(residual_data, output_path_ + "_residuals_" + ".txt");
    // list of time series of LHM-rleated methods
    if(obj_err_series.size() != 0)
        save2DdoubleVec(obj_err_series, output_path_ + "_obj_space_err_" + curr_sigma + "_.txt");
    if(quat_err_series.size() != 0)
        save2DdoubleVec(quat_err_series, output_path_ + "_rela_quat_err_" + curr_sigma + "_.txt");
    if(cos_diff_series.size() != 0)
        save2DdoubleVec(cos_diff_series, output_path_ + "_cos_diff_" + curr_sigma + "_.txt");
}
