#ifndef TEST_PNP_TEST_DATA_H_
#define TEST_PNP_TEST_DATA_H_

#include <Eigen/Core>
#include <unordered_map>
#include <string>
#include <memory>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"

enum class GeneratorType {
    COLMAP, BoxDz, CVLab, EPnPdZ, EPnPdY
};

inline GeneratorType getGeneratorFromName(const std::string& name) {
    static const std::unordered_map<std::string, GeneratorType> generatorMap = {
        {"colmap", GeneratorType::COLMAP},
        {"cv_lab", GeneratorType::CVLab},
        {"epnp_dz", GeneratorType::EPnPdZ},
        {"epnp_dy", GeneratorType::EPnPdY}
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
 * @brief derived data generator that 
 * simply copy the COLMAP's test data for EPnP and P3P
*/
class COLMAPTestData: public DataGenerator {
public:
    COLMAPTestData(){};

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
class BoxCornerEPnPTestData: public DataGenerator {
public:
    BoxCornerEPnPTestData(){};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
    static double sigma;
};


/**
 * @brief derived data generator from
 * EPFL CV lab's testing data
*/
class CVLabTestData: public DataGenerator {
public:
    CVLabTestData(){};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
private:
    std::string file_path = "/tmp3/Pose_PnP/LHM/";
};

/**
 * @brief generate a random rotation matrix and pass by reference
*/
void EPnPRandomRot(Eigen::Matrix3d& rot);

/**
 * @brief set the intrinsic matrix to convert the pixel to camera space
 * if use film plane data, set the intrinsic as an identity by default
*/
void SetIntrinsic(std::string calib_path, Eigen::Matrix3d& calib_mat);

/**
 * @brief the calibration file reader for the simulated data from EPFL CV-Lab
 * @arg calib_mat: the intrinsic matrix to be loaded
*/
void ReadCVLabCalib(std::string calib_path, Eigen::Matrix3d& calib_mat);

/**
 * @brief the 3d points reader for the simulated data from EPFL CV-Lab
*/
void ReadCVLab3D(std::string point3d_path, std::vector<Eigen::Vector3d>& points3D);

/**
 * @brief the 2d points reader for the simulated data from EPFL CV-Lab
*/
void ReadCVLab2D(std::string point2d_path, std::vector<Eigen::Vector2d>& points2D);

#endif // TEST_PNP_TEST_DATA_H_
