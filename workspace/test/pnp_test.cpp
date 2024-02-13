#include <Eigen/Core>
#include <string>
#include <memory>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "util/random.h"

#include "estimate/lhm.h"

#include "test/pnp_runner.h"
#include "test/pnp_test_data.h"
#include "test/pnp_test_template.h"

// hack test fn from absolute_pose_test.cpp
int main(int argc, char** argv) {
    std::string generator_opt = argv[1];
    std::string estimator_opt = argv[2];
    std::string use_ransac_ = argv[3];

    colmap::SetPRNGSeed(0);

    std::vector<Eigen::Vector3d> points3D;
    std::vector<Eigen::Vector2d> points2D;
    std::vector<Eigen::Matrix3x4d> poses;

    // Get the generator and estimator types from names
    GeneratorType gen_type = getGeneratorFromName(generator_opt);
    EstimatorType est_type = getEstimatorFromName(estimator_opt);

    EstimatorOptions options = EstimatorOptions();
    if(use_ransac_ == "1")
        options.use_ransac = true;
    std::cout << "current ransac option is: " << options.use_ransac << std::endl;

    // Create the data generator and estimator instances
    std::unique_ptr<DataGenerator> generator = DataGenerator::createDataGenerator(gen_type);
    EstimatorWrapper estimator(est_type, options); // Assuming EstimatorWrapper can be directly instantiated like this

    // Pass these instances to a TestRunner or another part of your application
    PnPTestRunner test_runner(std::move(generator), estimator);
    test_runner.run_test();
    
    return 0;
}



