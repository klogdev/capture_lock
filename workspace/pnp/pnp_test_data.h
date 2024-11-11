#ifndef PNP_PNP_TEST_DATA_H_
#define PNP_PNP_TEST_DATA_H_

#include <Eigen/Core>
#include <unordered_map>
#include <string>
#include <memory>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"
#include "file_reader/tum_rgbd.h"

enum class GeneratorType {
    EPnPdZ, EPnPdY, Outlier, PlanarChk, TUM, COLMAP, ORB,
    EPnPSimNoise, EPnPSimNum, EPnPSimOutlier
};

inline GeneratorType getGeneratorFromName(const std::string& name) {
    static const std::unordered_map<std::string, GeneratorType> generatorMap = {
        {"epnp_dz", GeneratorType::EPnPdZ},
        {"epnp_dy", GeneratorType::EPnPdY},
        {"outliers", GeneratorType::Outlier},
        {"planar_chk", GeneratorType::PlanarChk},
        {"tum_rgbd", GeneratorType::TUM},
        {"colmap_pair", GeneratorType::COLMAP},
        {"orb_gen", GeneratorType::ORB},
        {"epnp_sim_noise", GeneratorType::EPnPSimNoise},
        {"epnp_sim_num", GeneratorType::EPnPSimNum},
        {"epnp_sim_outlier", GeneratorType::EPnPSimOutlier}
    };

    auto it = generatorMap.find(name);
    if (it != generatorMap.end()) {
        return it->second;
    } else {
        throw std::invalid_argument("Unknown dataset");
    }
}

// virtual abstract class for the simulation data generator
// here we assume an array of (3D) scene points 
// has 1-to-1 corresponded 2D observation, i.e. a 2D vector of Eigen::Vector2d
class DataGenerator {
public:
    virtual void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D,
                          std::vector<std::vector<Eigen::Vector3d>>& points3D,
                          std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const = 0;
    static std::unique_ptr<DataGenerator> createDataGenerator(const GeneratorType type);
    virtual ~DataGenerator() = default;
};

/**
 * @brief data generator that 
 * we utilize EPnP's simulation data; the box corner [-2,2]x[-2,2]x[4,8]
 * originally definedin camera space; the intrinsic matrix is fixed as
 * f = 800, u, v = 320, 240 as specified in the article section 5.1
*/
class BoxCornerEPnPTestDataDz: public DataGenerator {
public:
    BoxCornerEPnPTestDataDz() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
    static double sigma;
};

class BoxCornerEPnPTestDataDy: public DataGenerator {
public:
    BoxCornerEPnPTestDataDy() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
    static double sigma;
};


/**
 * @brief varying percentage of outliers with fixed number
 * of random points inside the EPnP box
 */
class OutliersPercentage: public DataGenerator {
public:
    OutliersPercentage() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static double percent_s;
    static double percent_e;
};

/**
 * @brief interface class to load pre-aligned TUM-RGBD data
 * the g.t. poses are aligned with most closest timestamp of 
 * depth map data
 */
class TumRgbd: public DataGenerator {
public:
    TumRgbd() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
    
    static std::string image_parent;
    static std::string depth_parent;
    static std::string align_pose;
    static std::string curr_data;
};

/**
 * @brief interface class to load ORB-SLAM pricessed TUM-RGBD data
 */
class OrbGenerate: public DataGenerator {
public:
    OrbGenerate() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
    
    static std::string processed_orb;
};

class ColmapPair: public DataGenerator {

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static std::string processed_colmap;
};

/**
 * @brief sanity test with box corners or nearly planar cases
 * in this case we add noise directly to the 3D points
 * @arg option: the option to test planar or box corners
 */
class PlanarCase: public DataGenerator {
public:
    PlanarCase() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static double sigma_s;
    static double sigma_e;
    static bool tilt;
};

class EPnPSimulatorNoise: public DataGenerator {
public:
    EPnPSimulatorNoise() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static double sigma_s;
    static double sigma_e;
};

class EPnPSimulatorNumPts: public DataGenerator {
public:
    EPnPSimulatorNumPts() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static double sigma;
    static int min_pts;
    static int max_pts;
};

class EPnPSimulatorOutliers: public DataGenerator {
public:
    EPnPSimulatorOutliers() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static double sigma;
    static int min_pts;
    static int max_pts;
};


#endif // PNP_PNP_TEST_DATA_H_
