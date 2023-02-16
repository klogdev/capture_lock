#include "base/essential_matrix.h"
#include "base/reconstruction.h"
#include "base/camera.h"
#include "base/pose.h"
#include "base/triangulation.h"
#include "base/image.h"

#include "feature/sift.h"
#include "feature/image_sift.h"

#include "test_util.h"

#include "optim/ransac.h"
#include "estimators/pose.h"

const double PI = 3.1415926;

//call EstimateRelativePose from estimators/pose
size_t GetNumInliers(const colmap::RANSACOptions& ransac_options,
                    std::vector<Eigen::Vector2d>& points1,
                    std::vector<Eigen::Vector2d>& points2,
                    Eigen::Vector4d* qvec, Eigen::Vector3d* tvec){

    size_t num_inliers = 
        colmap::EstimateRelativePose(ransac_options, points1, points2, 
                                     qvec, tvec);
    return num_inliers;
}

int main(int argc, char** argv){
    if (argc < 4)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need two images' file path and colmap's file path" 
                  << std::endl;
    }

    //initialize the Image class by its path (feature/image_sift)
    Image image1(argv[1]);
    Image image2(argv[2]);
    //get matches from images
    std::vector<MatchedVec> Matches;
    
    Matches = FindMatches(image1, image2);

    std::vector<Eigen::Vector2d> key_points1;
    std::vector<Eigen::Vector2d> key_points2;

    for (auto match : Matches) {
        key_points1.push_back(match.KeyPt1);
        key_points2.push_back(match.KeyPt2);
    }
    //get file path for ReadText
    std::string file_path = argv[3];
    //read files and instantiate class for each
    colmap::Reconstruction read_text = colmap::Reconstruction();
    read_text.ReadText(file_path);

    //hard coded w/ specify camera 1
    colmap::Camera camera1 = read_text.Cameras().at(1);
    Eigen::Matrix3d calibration = camera1.CalibrationMatrix();

    //hard coded to get images P1180325/326
    //colmap::EIGEN_STL_UMAP(colmap::image_t, colmap::Image) image_map = read_text.Images();
    colmap::Image c_image1 = read_text.Image(34);
    colmap::Image c_image2 = read_text.Image(26);
    Eigen::Vector4d qvec1_gt = c_image1.Qvec();
    Eigen::Vector3d tvec1_gt = c_image1.Tvec();
    Eigen::Vector4d qvec2_gt = c_image2.Qvec();
    Eigen::Vector3d tvec2_gt = c_image2.Tvec();

    //get rotation matrix from ground truth pose
    Eigen::Matrix3d rot_mat_c1 = colmap::QuaternionToRotationMatrix(colmap::NormalizeQuaternion(qvec1_gt));
    Eigen::Matrix3d rot_mat_c2 = colmap::QuaternionToRotationMatrix(colmap::NormalizeQuaternion(qvec2_gt));

    //inverse the first rot matrix to identity, get relative pose of pose 2
    Eigen::Matrix3d rot_mat_inv1 = rot_mat_c1.inverse();
    Eigen::Matrix3d rot_mat_rel2 = rot_mat_inv1*rot_mat_c2;

    //get quaternion/trans of relative pose of image 2
    Eigen::Vector4d qvec_rel2 = colmap::RotationMatrixToQuaternion(rot_mat_rel2);
    Eigen::Vector3d tvec_rel2 = tvec2_gt - tvec1_gt;

    colmap::RANSACOptions ransac_options = colmap::RANSACOptions();
    ransac_options.max_error = 0.05;

    //init q&t vec for the first two frames
    Eigen::Vector4d qvec1 = Eigen::Vector4d(1, 0, 0, 0);
    Eigen::Vector3d tvec1 = Eigen::Vector3d::Zero();
    Eigen::Vector4d qvec2 = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d tvec2 = Eigen::Vector3d::Zero();

    //calculate d_quat and tvec norm for COLMAP g.t.
    Eigen::Quaterniond q_eigen1_gt;
    q_eigen1_gt.w() = qvec1_gt(0);
    q_eigen1_gt.x() = qvec1_gt(1);
    q_eigen1_gt.y() = qvec1_gt(2);
    q_eigen1_gt.z() = qvec1_gt(3);
    Eigen::Quaterniond rotated_q_eigen1gt = q_eigen1_gt.inverse();
    Eigen::Quaterniond q_eigen2_gt;
    q_eigen2_gt.w() = qvec2_gt(0);
    q_eigen2_gt.x() = qvec2_gt(1);
    q_eigen2_gt.y() = qvec2_gt(2);
    q_eigen2_gt.z() = qvec2_gt(3);
    
    Eigen::AngleAxisd aa_gt(q_eigen2_gt * rotated_q_eigen1gt);
    std::cout << "angle of delta_q: " << aa_gt.angle() * 180 / PI << std::endl;
    std::cout << "axis of dq: " << aa_gt.axis() << std::endl;
    double delta_t_gt = (tvec1_gt - tvec2_gt).norm();
    std::cout << "norm of tvec ground truth " << delta_t_gt << std::endl;

    Eigen::Matrix3x4d proj_mat1 = calibration*ProjMatFromQandT(qvec1, tvec1);

    size_t inliers = GetNumInliers(ransac_options,
                            key_points1, key_points2, &qvec2, &tvec2);

    //calculate d_quat and tvec norm for ransac estimate
    Eigen::Quaterniond q_eigen1;
    q_eigen1.w() = qvec1(0);
    q_eigen1.x() = qvec1(1);
    q_eigen1.y() = qvec1(2);
    q_eigen1.z() = qvec1(3);
    Eigen::Quaterniond rotated_q_eigen1 = q_eigen1.inverse();
    Eigen::Quaterniond q_eigen2;
    q_eigen2.w() = qvec2(0);
    q_eigen2.x() = qvec2(1);
    q_eigen2.y() = qvec2(2);
    q_eigen2.z() = qvec2(3);
    
    Eigen::AngleAxisd aa(q_eigen2 * rotated_q_eigen1);
    std::cout << "angle of delta_q: " << aa.angle() * 180 / PI << std::endl;
    std::cout << "axis of dq: " << aa.axis() << std::endl;
    double delta_t = (tvec1 - tvec2).norm();
    std::cout << "norm of tvec estimate " << delta_t << std::endl;

    //calculate error respect to g.t.
    std::cout << "squared error of qvec " << QvecSquareErr(qvec_rel2,qvec2) << std::endl;
    std::cout << "squared error of tvec " << TvecSquareErr(tvec_rel2,tvec2) << std::endl;

    Eigen::Matrix3x4d proj_mat2 = calibration*ProjMatFromQandT(qvec2, tvec2);

    std::vector<Eigen::Vector3d> points3d = colmap::TriangulatePoints(proj_mat1, proj_mat2,
                                                                    key_points1, key_points2);

    colmap::AbsolutePoseEstimationOptions absolute_options = colmap::AbsolutePoseEstimationOptions();
    absolute_options.ransac_options.max_error = 0.05;
    Eigen::Vector4d qvec_abs = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d tvec_abs = Eigen::Vector3d::Zero();
    std::vector<char> inlier_mask;
    bool abs_pose = colmap::EstimateAbsolutePose(absolute_options, key_points2,
                                                points3d, &qvec_abs, &tvec_abs,
                                                &camera1, &inliers, &inlier_mask);

    if (abs_pose){
        std::cout << "absolute rotation " << qvec_abs << std::endl;
        std::cout << "absolute translation " << tvec_abs << std::endl;
    }
    else {
        std::cout << "absolute pose estimate unsuccesful" << std::endl;
    }

    return 0;
}