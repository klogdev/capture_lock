#include <Eigen/Core>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"
#include "estimators/utils.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/lhm.h"
#include "estimate/least_sqr_pnp.h"

/**
 * @brief plain test of LHM/DRaM by using COLMAP's testing data
*/
int main(int argc, char** argv) {
        colmap::SetPRNGSeed(0);

        std::string qx_str = argv[1];
        std::string tx_str = argv[2];

        double qx_ = std::stod(qx_str);
        double tx_ = std::stod(tx_str);


        std::vector<Eigen::Vector3d> points3D;
        

        points3D.emplace_back(1, 1, 1);
        points3D.emplace_back(0, 1, 1);
        points3D.emplace_back(3, 1.0, 4);
        points3D.emplace_back(3, 1.1, 4);
        points3D.emplace_back(3, 1.2, 4);
        points3D.emplace_back(3, 1.3, 4);
        points3D.emplace_back(3, 1.4, 4);
        points3D.emplace_back(2, 1, 7);

        for (double qx = 0.0; qx < qx_; qx += 0.1) {
            for (double tx = 0; tx < tx_; tx += 0.1) {
                const colmap::SimilarityTransform3 orig_tform(1, Eigen::Vector4d(1, qx, 0, 0),
                                                            Eigen::Vector3d(tx, 0, 0));

                // Project points to camera coordinate system
                // here we project the original 3D points
                std::vector<Eigen::Vector2d> points2D;

                for (size_t i = 0; i < points3D.size(); i++) {
                    Eigen::Vector3d point3D_camera = points3D[i];
                    orig_tform.TransformPoint(&point3D_camera);

                    points2D.push_back(point3D_camera.hnormalized());
                }
            Eigen::Matrix3x4d manual_extrinsic; 
            LHMEstimator lhm;
            bool manual_lhm = lhm.ComputeLHMPose(points2D, points3D, 
                                            &manual_extrinsic);
            
            std::cout << "current g.t. pose is: " << std::endl;
            std::cout << orig_tform.Matrix().topLeftCorner<3, 4>() << std::endl;

            std::cout << "manually estimated lhm is: " << std::endl;
            std::cout << manual_extrinsic << std::endl;

            double matrix_diff =
                    (orig_tform.Matrix().topLeftCorner<3, 4>() - manual_extrinsic).norm(); 
            std::cout << "current frobenius norm is: " << matrix_diff << std::endl; 

            std::vector<double> res;
            colmap::ComputeSquaredReprojectionError(points2D, points3D, manual_extrinsic, &res);

            std::cout << "residuals are: " << std::endl;
            for(double r: res)
             std::cout << r << std::endl;
          }
        }
    }