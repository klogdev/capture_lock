#include <Eigen/Core>
#include <string>
#include <memory>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "util/random.h"

#include "estimate/lhm.h"
#include "estimate/least_sqr_pnp.h"

#include "pnp/pnp_runner.h"
#include "pnp/pnp_test_data.h"
#include "pnp/pnp_test_template.h"

int main(int argc, char** argv) {
    // args should specify the range of accuracy and the choice of estimator
    int iter_begin = std::stoi(argv[1]); // 5
    int iter_end = std::stoi(argv[2]);  // 60
    std::string estimator_opt = argv[3];

    EPnPSimulatorNoise::sigma_s = 0.5;
    EPnPSimulatorNoise::sigma_e = 0.5;

    LHMEstimator::options_.lhm_epsilon = 1e-15;
    LHMEstimator::options_.lhm_tolerance = 0.0;

    bool lhm_type = true;
    
    for(int i = iter_begin; i <= iter_end; i += 5) {
        
        // Get the generator and estimator types from names
        GeneratorType gen_type = getGeneratorFromName("random_noise");
        EstimatorType est_type = getEstimatorFromName(estimator_opt);

        EstimatorOptions options = EstimatorOptions();

        // Create the data generator and estimator instances
        std::unique_ptr<DataGenerator> generator = DataGenerator::createDataGenerator(gen_type);
        EstimatorWrapper estimator(est_type, options); // Assuming EstimatorWrapper can be directly instantiated like this

        LHMEstimator::options_.lhm_iter = i;
        // Set tolerance to a very large number
        LHMEstimator::options_.lhm_tolerance = 1e-9;


        // specify the output path for saving metrics
        // here we create new folder for each generator+estimator option
        std::string base_path = "/tmp3/Pose_PnP/PnP_result/";
        std::string output = base_path + "iteration_compare_" + std::to_string(EPnPSimulatorNoise::sigma_s) + "/" + estimator_opt;

        PnPTestRunner test_runner(std::move(generator), estimator, output, EPnPSimulatorNoise::sigma_s, lhm_type);

        test_runner.run_test();
    }
    return 0;
}