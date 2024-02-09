#ifndef TEST_PNP_RUNNER_H_
#define TEST_PNP_RUNNER_H_

#include <Eigen/Core>
#include <memory>

#include "test/pnp_test_template.h"
#include "test/pnp_test_data.h"

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

#endif // TEST_PNP_RUNNER_H_