#include <Eigen/Core>
#include <string>
#include <memory>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "util/random.h"

#include "estimate/lhm.h"
#include "estimate/least_sqr_pnp.h"

#include "test/pnp_runner.h"
#include "test/pnp_test_data.h"
#include "test/pnp_test_template.h"

int main(int argc, char** argv) {
    // args should specify the range of accuracy and the choice of estimator
    int exp_begin = std::stoi(argv[1]); // -3
    int exp_end = std::stoi(argv[2]);  // -9
    std::string estimator_opt = argv[3];

    BoxRandomEPnPTestDataNoise::sigma_s = 0.5;
    BoxRandomEPnPTestDataNoise::sigma_e = 0.5;

    bool lhm_type = true;
    
    for(int i = exp_begin; i >= exp_end; i--) {
        double error_threshold = std::pow(10, i);
        
        // Get the generator and estimator types from names
        GeneratorType gen_type = getGeneratorFromName("random_noise");
        EstimatorType est_type = getEstimatorFromName(estimator_opt);

        EstimatorOptions options = EstimatorOptions();

        // Create the data generator and estimator instances
        std::unique_ptr<DataGenerator> generator = DataGenerator::createDataGenerator(gen_type);
        EstimatorWrapper estimator(est_type, options); // Assuming EstimatorWrapper can be directly instantiated like this

        // estimator.options_.lhm_opt.lhm_epsilon = error_threshold;
        // estimator.options_.lhm_opt.lhm_iter = std::numeric_limits<int>::max();
        LHMEstimator::options_.lhm_epsilon = error_threshold;
        LHMEstimator::options_.lhm_iter = std::numeric_limits<int>::max();

        // specify the output path for saving metrics
        // here we create new folder for each generator+estimator option
        std::string base_path = "/tmp3/Pose_PnP/PnP_result/";
        std::string output = base_path + "iteration_compare_" + std::to_string(BoxRandomEPnPTestDataNoise::sigma_s) + "/" + estimator_opt;

        PnPTestRunner test_runner(std::move(generator), estimator, output, BoxRandomEPnPTestDataNoise::sigma_s, lhm_type);

        test_runner.run_test();
    }
    return 0;
}