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
                  EstimatorWrapper estimator, 
                  std::string output_path, double sigma = 0.0003,
                  bool lhm_type = false)
        : data_generator_(std::move(data_generator)),
          estimator_(estimator), output_path_(output_path),
          sigma_(sigma), lhm_type_(lhm_type) {}

    void run_test();

private:
    std::unique_ptr<DataGenerator> data_generator_;
    EstimatorWrapper estimator_;
    std::string output_path_;
    double sigma_;
    bool lhm_type_;
};


#endif // TEST_PNP_RUNNER_H_