
#include <Eigen/Core>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

struct EstimatorOptions {
    bool useRansac = false; // Default to standalone estimation
};

class EstimatorWrapper {
public:
    enum EstimatorType { EPnP, LHM, LHM_DRaM };

    EstimatorWrapper(EstimatorType type, const EstimatorOptions& options)
        : type_(type), options_(options) {}


    bool estimate(const std::vector<Eigen::Vector2d>& points2D,
                  const std::vector<Eigen::Vector3d>& points3D,
                  Eigen::Matrix3x4d& estimatedExtrinsic,
                  std::vector<double>* residuals = nullptr);

private:
    EstimatorType type_;
    EstimatorOptions options_;

    /**
     * @brief runs standalone estimator
    */
    bool runStandalone(const std::vector<Eigen::Vector2d>& points2D,
                        const std::vector<Eigen::Vector3d>& points3D,
                        Eigen::Matrix3x4d& estimatedExtrinsic,
                        std::vector<double>* residuals);

    /**
     * @brief runs the estimator w/ RANSAC
    */
    bool runWithRansac(const std::vector<Eigen::Vector2d>& points2D,
                        const std::vector<Eigen::Vector3d>& points3D,
                        Eigen::Matrix3x4d& estimatedExtrinsic,
                        std::vector<double>* residuals);
};
