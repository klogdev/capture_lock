#include <Eigen/Core>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

class DataGenerator {
public:
    virtual void generate(std::vector<Eigen::Vector2d>& points2D,
                          std::vector<Eigen::Vector3d>& points3D,
                          Eigen::Matrix3x4d& extrinsic) = 0;
    virtual ~DataGenerator() {}
};


/**
 * @brief simply copy the COLMAP's test data for EPnP and P3P
*/
void COLMAPTestData(std::vector<Eigen::Vector2d>& points2D, 
                    std::vector<Eigen::Vector3d>& points3D,
                    std::vector<Eigen::Matrix3x4d> composed_extrinsic);