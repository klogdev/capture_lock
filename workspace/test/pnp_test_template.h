#ifndef TEST_PNP_TEST_TEMPLATE_H_
#define TEST_PNP_TEST_TEMPLATE_H_

#include <Eigen/Core>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

enum class EstimatorType {EPnP, DLS, LHM, DRaM_LHM, DRaM_GN};

inline EstimatorType getEstimatorFromName(const std::string& name) {
    static const std::unordered_map<std::string, EstimatorType> estimatorMap = {
        {"dram_lhm", EstimatorType::DRaM_LHM},
        {"epnp", EstimatorType::EPnP},
        {"lhm", EstimatorType::LHM}
    };

    auto it = estimatorMap.find(name);
    if (it != estimatorMap.end()) {
        return it->second;
    } else {
        throw std::invalid_argument("Unknown dataset");
    }
}


struct EstimatorOptions {
    bool use_ransac = false; // Default to standalone estimation
    Eigen::Matrix3x4d* gt_pose = nullptr; 
    LHMOptions lhm_opt = LHMOptions();

    // Constructor for when you want to set the ground truth pose
    EstimatorOptions(bool use_ransac_ = false, Eigen::Matrix3x4d* gt_pose_ = nullptr,
                    LHMOptions lhm_option = LHMOptions())
        : use_ransac(use_ransac), gt_pose(gt_pose), lhm_opt(lhm_option) {}
};

// we implement two separate template for PnP
// one is LHM and its variants, which need init extra static global variables
template<typename Estimator>
class LHMEstimatorWrapper {
public:
    typedef typename Estimator::X_t X_t;
    typedef typename Estimator::Y_t Y_t;
    typedef typename Estimator::M_t M_t;


    LHMEstimatorWrapper(EstimatorType type, const EstimatorOptions& options)
        : type_(type), options_(options) {}

    /**
     * @brief main method as an interface to do PnP estimation
     * @arg residuals by COLMAP's RANSAC template's default, is a pointer
    */
    bool estimate(const std::vector<X_t>& points2D,
                  const std::vector<Y_t>& points3D,
                  M_t& estimated_extrinsic,
                  std::vector<double>* residuals = nullptr);

private:
    EstimatorType type_;
    EstimatorOptions options_;

    /**
     * @brief runs standalone estimator
    */
    bool runStandalone(const std::vector<X_t>& points2D,
                       const std::vector<Y_t>& points3D,
                       M_t& estimated_extrinsic,
                       std::vector<double>* residuals);

    /**
     * @brief runs the estimator w/ RANSAC
    */
    bool runWithRansac(const std::vector<X_t>& points2D,
                       const std::vector<Y_t>& points3D,
                       M_t& estimatedExtrinsic,
                       std::vector<double>* residuals);
};

// we implement two separate template for PnP
// the second is the plain PnP, such as EPnP, DLS et.al
template<typename Estimator>
class EstimatorWrapper {
public:
    typedef typename Estimator::X_t X_t;
    typedef typename Estimator::Y_t Y_t;
    typedef typename Estimator::M_t M_t;


    EstimatorWrapper(EstimatorType type, const EstimatorOptions& options)
        : type_(type), options_(options) {}

    /**
     * @brief main method as an interface to do PnP estimation
     * @arg residuals by COLMAP's RANSAC template's default, is a pointer
    */
    bool estimate(const std::vector<X_t>& points2D,
                  const std::vector<Y_t>& points3D,
                  M_t& estimated_extrinsic,
                  std::vector<double>* residuals = nullptr);

private:
    EstimatorType type_;
    EstimatorOptions options_;

    /**
     * @brief runs standalone estimator
    */
    bool runStandalone(const std::vector<X_t>& points2D,
                       const std::vector<Y_t>& points3D,
                       M_t& estimated_extrinsic,
                       std::vector<double>* residuals);

    /**
     * @brief runs the estimator w/ RANSAC
    */
    bool runWithRansac(const std::vector<X_t>& points2D,
                       const std::vector<Y_t>& points3D,
                       M_t& estimatedExtrinsic,
                       std::vector<double>* residuals);
};

#endif // TEST_PNP_TEST_TEMPLATE_H_