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
#include "file_reader/tum_rgbd.h"

enum class GeneratorType {
    COLMAP, CVLab, EPnPdZ, EPnPdY, RandomNoise, NumPts, Outlier, 
    PlanarChk, TUM
};

inline GeneratorType getGeneratorFromName(const std::string& name) {
    static const std::unordered_map<std::string, GeneratorType> generatorMap = {
        {"cv_lab", GeneratorType::CVLab},
        {"epnp_dz", GeneratorType::EPnPdZ},
        {"epnp_dy", GeneratorType::EPnPdY},
        {"random_noise", GeneratorType::RandomNoise},
        {"num_pts", GeneratorType::NumPts},
        {"outliers", GeneratorType::Outlier},
        {"planar_chk", GeneratorType::PlanarChk},
        {"tum_rgbd", GeneratorType::TUM}
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
 * @brief use EPnP centered box to generate random data inside it
 * with different level of noises
 */
class BoxRandomEPnPTestDataNoise: public DataGenerator {
public:
    BoxRandomEPnPTestDataNoise() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
    static double sigma_s; // start sigma
    static double sigma_e; // end sigma
    static int num_pts;
};

/**
 * @brief varying number of control points inside the box
 * with a fixed noise sigma
 */
class BoxRandomTestNumPts: public DataGenerator {
public:
    BoxRandomTestNumPts() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
    static double sigma;
    static int min_pts;
    static int max_pts;
};

/**
 * @brief varying percentage of outliers with fixed number
 * of random points inside the EPnP box
 */
class BoxRandomOutliers: public DataGenerator {
public:
    BoxRandomOutliers() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static double percent_s;
    static double percent_e;
};

/**
 * @brief sanity test with box corners or nearly planar cases
 * in this case we add noise directly to the 3D points
 * @arg option: the option to test planar or box corners
 */
class BoxCornerPlanarSanity: public DataGenerator {
public:
    BoxCornerPlanarSanity() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;

    static double sigma_s;
    static double sigma_e;
    static std::string option;
};

class TumRgbd: public DataGenerator {
public:
    TumRgbd() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
    
    static std::string depth_parent;
    static std::string align_pose;
};

/**
 * @brief derived data generator from
 * EPFL CV lab's testing data
*/
class CVLabTestData: public DataGenerator {
public:
    CVLabTestData() {};

    void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D, 
                  std::vector<std::vector<Eigen::Vector3d>>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override;
private:
    std::string file_path = "/tmp3/Pose_PnP/LHM/";
};

/**
 * @brief preprocessing of the EPnP box corner data generation
 * @arg  3D points in camera space
*/
void EPnPBoxCorner(std::vector<Eigen::Vector3d>& camera_space_points);

/**
 * @brief 
 * preprocessing of a planar data generation comparable to EPnP box corners
 * @arg 3D points in camera space
 */
void EPnPPlanar(std::vector<Eigen::Vector3d>& camara_space_points);

/**
 * @brief pass the intrinsic matrix by reference
 */
void GetIntrinsic(Eigen::Matrix3d& k);

/**
 * @brief preprocessing of the EPnP random data generation
 * inside the box [-2,2]x[-2,2]x[4,8]
*/
void EPnPInsideRand(std::vector<Eigen::Vector3d>& camera_space_points,
                    int num_pts);

/**
 * @brief generate a random rotation matrix and pass by reference
*/
void EPnPRandomRot(Eigen::Matrix3d& rot);

/**
 * @brief generate a random translation vector and pass by reference
 */
void EPnPRandomTrans(Eigen::Vector3d& trans);

/**
 * @brief translate points inside the camera space
 */
void CameraSpaceShift(const std::vector<Eigen::Vector3d>& camera_pts, 
                      const Eigen::Vector3d& trans,
                      std::vector<Eigen::Vector3d>& shifted_pts);

/**
 * @brief set the intrinsic matrix to convert the pixel to camera space
 * if use film plane data, set the intrinsic as an identity by default
*/
void SetIntrinsic(std::string calib_path, Eigen::Matrix3d& calib_mat);

/**
 * @brief generate noised 2d set from a list of camera space points
 */
void GenOneSetNoise2D(std::vector<Eigen::Vector3d>& camera_space_points, 
                      std::vector<Eigen::Vector2d>& one_set_2d,
                      Eigen::Matrix3d& k, double sigma);

/**
 * @brief add small perturbation to each camera space point
 */
void Perturbation3D(std::vector<Eigen::Vector3d>& camera_space_points,
                    double sigma);

/**
 * @brief replace part of 2D points as outliers
 * we follow the EPnP data where the aspect ratio for image is 640, 480
 */
void AddOutlier2D(std::vector<Eigen::Vector2d>& points2D, double outlier_rate, 
                  const int image_x, const int image_y);


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
