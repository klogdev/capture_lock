#include <Eigen/Core>

#include "base/essential_matrix.h"
#include "base/pose.h"

#include "estimators/pose.h"
#include "estimators/essential_matrix.h"
#include "estimators/fundamental_matrix.h"

#include "optim/ransac.h"

#include "feature/sift.h"
#include "feature/image_sift.h"

#include "util_/math_helper.h"
#include "util_/sift_colmap.h"

#include "file_reader/file_options.h"


int main(int argc, char** argv){
    if (argc < 4)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need two images file path" << std::endl;
    }

    std::string datasetName = argv[3];

    Dataset dataset = getDatasetFromName(datasetName);

    // 1. instantiate file options based on file type, 
    // @file_reader/file_option.h
    FileOptions files_to_run;
    InstantiateFiles(files_to_run, dataset);

    // 2. factory pattern to load calibration info
    std::unique_ptr<CalibFileReader> file_reader = CalibFileReader::CalibFileCreate(dataset);
    colmap::Camera camera = file_reader->GetIntrinsicMat(files_to_run.calib_path,
                                                         files_to_run.seq_num,
                                                         files_to_run.downsample,
                                                         files_to_run.width,
                                                         files_to_run.height);

    // initialize the Image class by its path (feature/image_sift)
    Image image1(argv[1], 1242, 375); // we assume they are kitti data
    Image image2(argv[2], 1242, 375);

    std::vector<sift::Keypoint> key_points1 = GetKeyPoints(image1);
    std::vector<sift::Keypoint> key_points2 = GetKeyPoints(image2);
    std::vector<std::pair<int, int>> matches = sift::find_keypoint_matches(key_points1, key_points2);

    // convert to feature corrdinates 
    std::vector<Eigen::Vector2d> points1 = SIFTPtsToVec(key_points1);
    std::vector<Eigen::Vector2d> points2 = SIFTPtsToVec(key_points2);

    // start two view geometry estimation
    std::vector<Eigen::Vector2d> homo1;
    std::vector<Eigen::Vector2d> homo2;

    for(int i = 0; i < points1.size(); i++) {
        homo1.push_back(camera.ImageToWorld(points1[i]));
        homo2.push_back(camera.ImageToWorld(points2[i]));
    }
    int start = 0;
    int end = 16;
    std::vector<Eigen::Vector2d> sl_homo1(homo1.begin() + start, homo1.begin() + end);
    std::vector<Eigen::Vector2d> sl_homo2(homo2.begin() + start, homo2.begin() + end);
    std::vector<Eigen::Vector2d> sl_pots1(points1.begin() + start, points1.begin() + end);
    std::vector<Eigen::Vector2d> sl_pots2(points2.begin() + start, points2.begin() + end);

    // essential matrix
    colmap::RANSACOptions E_options = colmap::RANSACOptions();
    E_options.max_error = 0.5;
    colmap::RANSAC<colmap::EssentialMatrixFivePointEstimator> e_ransac(E_options);
    const auto e_report = e_ransac.Estimate(sl_homo1, sl_homo2);
    Eigen::Matrix3d E = e_report.model;
    std::cout << "pass 5-points" << std::endl;
    // fundamental matrix
    colmap::RANSACOptions F_options = colmap::RANSACOptions();
    F_options.max_error = 0.5;
    colmap::RANSAC<colmap::FundamentalMatrixSevenPointEstimator> f_ransac(F_options);
    std::cout << "before 7-points" << std::endl;
    const auto f_report = f_ransac.Estimate(sl_pots1, sl_pots2);
    Eigen::Matrix3d F = f_report.model;
    std::cout << "pass 7-points" << std::endl;

    // check results
    Eigen::Matrix3d k = camera.CalibrationMatrix();
    Eigen::Matrix3d k_t = k.transpose();

    std::cout << "estimated essential: " << std::endl;
    std::cout << E << std::endl;
    std::cout << "estimated converted fundamental: " << std::endl;
    std::cout << k_t*F*k << std::endl;

    double frob = frobeniusNormRot(E, k_t*F*k);
    std::cout << "frob between them is: " << frob << std::endl;
}