#ifndef PNP_PNP_TEST_TEMPLATE_H_
#define PNP_PNP_TEST_TEMPLATE_H_

#include <Eigen/Core>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

using X_t = Eigen::Vector2d;
using Y_t = Eigen::Vector3d;
using M_t = Eigen::Matrix3x4d;


enum class EstimatorType {EPnP, DLS, LHM, DRaM_LHM, EPnP_Colmap};

inline EstimatorType getEstimatorFromName(const std::string& name) {
    static const std::unordered_map<std::string, EstimatorType> estimatorMap = {
        {"dram_lhm", EstimatorType::DRaM_LHM},
        {"epnp", EstimatorType::EPnP},
        {"lhm", EstimatorType::LHM},
        {"dls", EstimatorType::DLS},
        {"epnp_colmap", EstimatorType::EPnP_Colmap}
    };

    auto it = estimatorMap.find(name);
    if (it != estimatorMap.end()) {
        return it->second;
    } else {
        throw std::invalid_argument("Unknown estimator");
    }
}

struct EstimatorOptions {
    int use_ransac = 1; // Default to standalone estimation, 1 will be ransac, 2 will be loransac
};

class EstimatorWrapper {
public:
    EstimatorWrapper(EstimatorType type, const EstimatorOptions& options)
        : type_(type), options_(options) {}

    /**
     * @brief main method as an interface to do PnP estimation
     * @arg residuals: by COLMAP's RANSAC template's default, it is a pointer
     * estimated_extrinsic: by COLMAP's default, it returns a vector
     * of matrix, as estimator like P3P will result in ambiguity, i.e. multiple solutions
    */
    bool estimate(const std::vector<Eigen::Vector2d>& points2D,
                  const std::vector<Eigen::Vector3d>& points3D,
                  std::vector<Eigen::Matrix3x4d>& estimated_extrinsic,
                  std::vector<double>* residuals = nullptr);

private:
    EstimatorType type_;
    EstimatorOptions options_;

    /**
     * @brief runs standalone estimator
    */
    bool runStandalone(const std::vector<Eigen::Vector2d>& points2D,
                       const std::vector<Eigen::Vector3d>& points3D,
                       std::vector<Eigen::Matrix3x4d>& estimated_extrinsic,
                       std::vector<double>* residuals);

    /**
     * @brief runs the estimator w/ RANSAC
    */
    bool runWithRansac(const std::vector<Eigen::Vector2d>& points2D,
                       const std::vector<Eigen::Vector3d>& points3D,
                       std::vector<Eigen::Matrix3x4d>& estimatedExtrinsic,
                       std::vector<double>* residuals);

    /**
     * @brief runs the estimators w/ LO-RANSAC
     */
    bool runWithLoRansac(const std::vector<Eigen::Vector2d>& points2D,
                         const std::vector<Eigen::Vector3d>& points3D,
                         std::vector<Eigen::Matrix3x4d>& estimatedExtrinsic,
                         std::vector<double>* residuals);
};


#endif // PNP_PNP_TEST_TEMPLATE_H_