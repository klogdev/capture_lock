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
    COLMAP
};

inline GeneratorType getGeneratorFromName(const std::string& name) {
    static const std::unordered_map<std::string, GeneratorType> generatorMap = {
        {"colmap", GeneratorType::COLMAP}
    };

    auto it = generatorMap.find(name);
    if (it != generatorMap.end()) {
        return it->second;
    } else {
        throw std::invalid_argument("Unknown dataset");
    }
}

// virtual bastract class for the simulation data generator
// here we assume a single array of (3D) scene points 
// has multiview 2D observation, i.e. a 2D vector of Eigen::Vector2d
class DataGenerator {
public:
    virtual void generate(std::vector<std::vector<Eigen::Vector2d>>& points2D,
                          std::vector<Eigen::Vector3d>& points3D,
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
                  std::vector<Eigen::Vector3d>& points3D,
                  std::vector<Eigen::Matrix3x4d>& composed_extrinsic) const override {
        points3D.emplace_back(1, 1, 1);
        points3D.emplace_back(0, 1, 1);
        points3D.emplace_back(3, 1.0, 4);
        points3D.emplace_back(3, 1.1, 4);
        points3D.emplace_back(3, 1.2, 4);
        points3D.emplace_back(3, 1.3, 4);
        points3D.emplace_back(3, 1.4, 4);
        points3D.emplace_back(2, 1, 7);

        for (double qx = 0; qx < 0.2; qx += 0.2) {
            
            std::vector<Eigen::Vector2d> curr_points2D;
            for (double tx = 0; tx < 0.2; tx += 0.1) {
                const colmap::SimilarityTransform3 orig_tform(1, Eigen::Vector4d(1, qx, 0, 0),
                                                            Eigen::Vector3d(tx, 0, 0));

                // Project points to camera coordinate system
                // here we project the original 3D points
                for (size_t i = 0; i < points3D.size(); i++) {
                    Eigen::Vector3d point3D_camera = points3D[i];
                    orig_tform.TransformPoint(&point3D_camera);
                    curr_points2D.push_back(point3D_camera.hnormalized());
                }

                std::cout << "current rotation simulation is: " << qx << std::endl;
                std::cout << "current translation simulation is: " << tx << std::endl;
                std::cout << "current transform is: " << std::endl;
                std::cout << orig_tform.Matrix() << std::endl;
                composed_extrinsic.push_back(orig_tform.Matrix().topLeftCorner<3, 4>());
            }
            points2D.push_back(curr_points2D);
        }
    }
};

#endif // TEST_PNP_TEST_DATA_H_
