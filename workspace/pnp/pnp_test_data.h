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
    EPnPdZ, EPnPdY, Outlier, PlanarChk, PlanarPtb, BoxPtb,
    TUM, COLMAP, ORB,
    EPnPSimNoise, EPnPSimNum, EPnPSimOutlier,
    POSITCube
};

inline GeneratorType getGeneratorFromName(const std::string& name) {
    static const std::unordered_map<std::string, GeneratorType> generatorMap = {
        {"epnp_dz", GeneratorType::EPnPdZ}, // already updated as translation relative to camera origin
        {"epnp_dy", GeneratorType::EPnPdY},
        {"outliers", GeneratorType::Outlier}, // what we are using now
        {"planar_chk", GeneratorType::PlanarChk},
        {"planar_ptb", GeneratorType::PlanarPtb},
        {"box_ptb", GeneratorType::BoxPtb},
        {"tum_rgbd", GeneratorType::TUM},
        {"colmap_pair", GeneratorType::COLMAP},
        {"orb_gen", GeneratorType::ORB},
        {"epnp_sim_noise", GeneratorType::EPnPSimNoise},
        {"epnp_sim_num", GeneratorType::EPnPSimNum},
        {"posit_cube", GeneratorType::POSITCube}
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

class PositCube : public DataGenerator
{
public:
    PositCube() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
};


/**
 * @brief data generator that 
 * we utilize EPnP's simulation data; the box corner [-2,2]x[-2,2]x[4,8]
 * originally definedin camera space; the intrinsic matrix is fixed as
 * f = 800, u, v = 320, 240 as specified in the article section 5.1
*/
class BoxCornerEPnPDataDz: public DataGenerator {
public:
    BoxCornerEPnPDataDz() {};

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

    static bool add_outliers;
    static double outliers_percent;
    static int num_repeat; // repeat for every frame
    static int num_each_frame; // number of points to be sampled in each frame
    static int seq_num; // can be tum, freiburg1, freiburg2 or euroc mav's v01
};

/**
 * @brief we have separate colmap generated data
 * cause we preprocessed the 2d-3d pairs with undistorted & normalized 
 * image coordinates
 * @arg processed_colmap: the path to the processed colmap data
 * @arg radius: the radius from principal point (normalized, i.e. 0, 0) 
 * to filter the points
 */
class ColmapPair: public DataGenerator {

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static std::string processed_colmap;
    static double radius_small; 
    static double radius_large;
    static int num_repeat;
    static int num_each_frame;
};

/**
 * @brief sanity test with nearly planar cases
 * in this case we keep constant number of points, usually 50
 * then move part of point out of the plane, the number are ranges
 * from 1 to 10
 * @arg vertices_s: the start number of vertices out of the plane
 */
class PlanarCase: public DataGenerator {
public:
    PlanarCase() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static double vertices_s; // number of vertices out of the plane
    static double vertices_e;
    static bool tilt;
};

/**
 * @brief sanity test with nearly planar cases
 * in this case we add noise directly to the 3D points
 * @arg option: the option to test planar or box corners
 */
class PlanarPerturb: public DataGenerator {
public:
    PlanarPerturb() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static double sigma_s; // number of vertices out of the plane
    static double sigma_e;
    static bool tilt;
};

/**
 * @brief sanity test with box corners cases
 * in this case we add noise directly to the 3D points
 * @arg option: the option to test planar or box corners
 */
class BoxPerturb: public DataGenerator {
public:
    BoxPerturb() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static double sigma_s; // number of vertices out of the plane
    static double sigma_e;
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

#endif // PNP_PNP_TEST_DATA_H_
