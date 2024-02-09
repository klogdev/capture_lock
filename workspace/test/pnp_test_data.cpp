#include <Eigen/Core>
#include <memory>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

#include "test/pnp_test_data.h"

std::unique_ptr<DataGenerator> 
DataGenerator::createDataGenerator(const GeneratorType type) {
    switch (type) {
        case GeneratorType::COLMAP:
            return std::make_unique<COLMAPTestData>(COLMAPTestData());
        // Handle unsupported types
        default:
            return nullptr;
    }
}

