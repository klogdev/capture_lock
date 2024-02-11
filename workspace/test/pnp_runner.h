#ifndef TEST_PNP_RUNNER_H_
#define TEST_PNP_RUNNER_H_

#include <Eigen/Core>
#include <memory>

#include "test/pnp_test_template.h"
#include "test/pnp_test_data.h"

#include "estimate/lhm.h"

class PnPTestRunner {
public:
    PnPTestRunner(std::unique_ptr<DataGenerator> data_generator,
               EstimatorWrapper estimator)
        : data_generator_(std::move(data_generator)),
          estimator_(estimator) {}

    void run_test();

private:
    std::unique_ptr<DataGenerator> data_generator_;
    EstimatorWrapper estimator_;
};

/**
 * @brief convert plain type of Eigen to the customized COLMAP's type
 * for the Estimator can use the data from Generator
*/
template<typename Estimator>
void convertData(const std::vector<std::vector<Eigen::Vector2d>>& points2D,
                 std::vector<std::vector<typename Estimator::X_t>>& converted_points2D);

template<typename Estimator>
void convertData(const std::vector<Eigen::Vector3d>& points3D,
                 std::vector<typename Estimator::Y_t>& converted_points3D);

template<typename Estimator>
void convertData(const std::vector<Eigen::Matrix3x4d>& extrinsic,
                 std::vector<typename Estimator::M_t>& converted_extrinsic);

#endif // TEST_PNP_RUNNER_H_