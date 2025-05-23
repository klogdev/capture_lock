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

// hack test fn from absolute_pose_test.cpp
int main(int argc, char** argv) {
    std::string generator_opt = argv[1];
    std::string estimator_opt = argv[2];
    std::string use_ransac_ = argv[3];

    // in runner, sigma only responsible for file naming
    // so we set it as -1 as a dummy value
    double sigma = -1.0;

    // Handle the 4th argument differently based on generator type
    if(argc >= 5) {
        if(generator_opt == "orb_gen") {
            // For orb_gen, use the argument directly as path
            OrbGenerate::processed_orb = argv[4];
        } else if(generator_opt == "epnp_dz" || generator_opt == "epnp_dy") {
            // For other generators that need numeric sigma
            try {
                double sigma = std::stod(argv[4]);
                if(generator_opt == "epnp_dz") {
                    BoxCornerEPnPDataDz::sigma = sigma;
                } else if(generator_opt == "epnp_dy") {
                    BoxCornerEPnPTestDataDy::sigma = sigma;
                }
            } catch(const std::invalid_argument& e) {
                std::cerr << "Error: Expected numeric value for sigma with " << generator_opt << std::endl;
                return 1;
            } catch(const std::out_of_range& e) {
                std::cerr << "Error: Sigma value out of range" << std::endl;
                return 1;
            }
        }
    }

    if(argc >= 6) {
        if(generator_opt != "orb_gen") {
            std::cerr << "Error: Expected 'orb_gen' generator type for argument more than 5" << std::endl;
            return 1;
        }
        OrbGenerate::num_each_frame = std::stoi(argv[5]);
        OrbGenerate::num_repeat = std::stoi(argv[6]);
        OrbGenerate::seq_num = std::stoi(argv[7]);
    }
    
    bool lhm_type = false;
    if(estimator_opt == "lhm" || estimator_opt == "dram_lhm") {
        lhm_type = true;
    }

    colmap::SetPRNGSeed(0);

    // Get the generator and estimator types from names
    GeneratorType gen_type = getGeneratorFromName(generator_opt);
    EstimatorType est_type = getEstimatorFromName(estimator_opt);

    EstimatorOptions options = EstimatorOptions();
    
    options.use_ransac = std::stoi(use_ransac_);  
    std::cout << "current ransac option is: " << options.use_ransac << std::endl;

    // Create the data generator and estimator instances
    std::unique_ptr<DataGenerator> generator = DataGenerator::createDataGenerator(gen_type);
    EstimatorWrapper estimator(est_type, options); // Assuming EstimatorWrapper can be directly instantiated like this

    // specify the output path for saving metrics
    // here we create new folder for each generator+estimator option
    std::string outlier_opt = "";
    std::string output = "/tmp3/Pose_PnP/PnP_result/" + generator_opt + "_" + std::to_string(sigma) + "/" + estimator_opt;

    // make the output folder for data from ORBSLAM different if add outlier
    if(options.use_ransac) {
        if(generator_opt == "orb_gen") {
            outlier_opt = "_outlier";
            output = "/tmp3/Pose_PnP/PnP_result/" + generator_opt + "_" + std::to_string(sigma) + outlier_opt + "/" + estimator_opt;
        }
    }
    // Pass these instances to a TestRunner or another part of your application

    PnPTestRunner test_runner(std::move(generator), estimator, output, sigma, lhm_type);
    test_runner.run_test();
    
    return 0;
}
