#include <Eigen/Core>

#include "estimators/absolute_pose.h"
#include "estimators/pose.h"
#include "estimators/utils.h"

#include "base/similarity_transform.h"
#include "util/random.h"

#include "estimate/adj_quat.h"
#include "util_/math_helper.h"

#include "test/horn_test.h"

int main(int argc, char** argv) {
        colmap::SetPRNGSeed(0);

        std::string qx_str = argv[1];
        std::string tx_str = argv[2];

        double qx = std::stod(qx_str);
        double tx = std::stod(tx_str);

        std::vector<Eigen::Vector3d> points3D;

        points3D.emplace_back(1, 1, 1);
        points3D.emplace_back(0, 1, 1);
        points3D.emplace_back(3, 1.0, 4);
        points3D.emplace_back(3, 1.1, 4);
        points3D.emplace_back(3, 1.2, 4);
        points3D.emplace_back(3, 1.3, 4);
        points3D.emplace_back(3, 1.4, 4);
        points3D.emplace_back(2, 1, 7);

        const colmap::SimilarityTransform3 orig_tform(1, Eigen::Vector4d(1, qx, 0, 0),
                                                    Eigen::Vector3d(tx, 0, 0));

        std::vector<Eigen::Vector3d> points3D_1;
        std::vector<Eigen::Vector2d> points2D_1;
        std::vector<Eigen::Vector3d> points3D_homo;

        for (size_t i = 0; i < points3D.size(); i++) {
            Eigen::Vector3d point3D_copy = points3D[i];
            orig_tform.TransformPoint(&point3D_copy);
            points3D_1.push_back(point3D_copy);
            Eigen::Vector2d curr_2d = point3D_copy.hnormalized();
            points2D_1.push_back(curr_2d);

            Eigen::Vector3d curr_homo(curr_2d[0], curr_2d[1], 1.0);
            points3D_homo.push_back(curr_homo);
            }
        Eigen::Matrix3d gt_rot = orig_tform.Matrix().topLeftCorner<3, 3>();

        Eigen::Matrix3d R;
        bool rot_test = HornRot(points3D, points3D_1, R);
        double frob_R = frobeniusNorm(R, gt_rot);
        std::cout << "estimated matrix by Horn is: " << std::endl;
        std::cout << R << std::endl;
        std::cout << "the Frob norm for Horn's 3D reg is: " << frob_R << std::endl;

        Eigen::Matrix3d R_weak;
        bool weak = HornRot(points3D, points3D_homo, R_weak);
        double frob_Rw = frobeniusNorm(R_weak, gt_rot);
        std::cout << "estimated matrix by Horn for the weak perspective is: " << std::endl;
        std::cout << R_weak << std::endl;
        std::cout << "the Frob norm for Horn's weak perpective is: " << frob_Rw << std::endl;


        Eigen::Matrix3d R_bi;
        bool bi_rot = BarItzhackOptRot(points3D, points2D_1, R_bi);
        double frob_Rbi = frobeniusNorm(R_bi, gt_rot);
        std::cout << "estimated matrix by DRaM for the weak perspective is: " << std::endl;
        std::cout << R_bi << std::endl;
        std::cout << "the Frob norm for DRaM's weak perpective is: " << frob_Rbi << std::endl;

}