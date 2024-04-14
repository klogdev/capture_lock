#include <Eigen/Core>
#include <iterator>

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

void LoadCOLMAPLog(std::string path, 
                   std::vector<Eigen::Vector2d>& load_feat1, std::vector<Eigen::Vector2d>& load_feat2,
                   std::vector<Eigen::Vector2d>& load_norm1, std::vector<Eigen::Vector2d>& load_norm2) {
    
    std::ifstream colmap_log(path);
    std::string line;

    while(std::getline(colmap_log, line)){
        std::istringstream iss(line);
        // the first iterator process the "line", the second is the dummy one as
        // an end iterator
        std::vector<std::string> curr_line{std::istream_iterator<std::string>{iss}, 
                                           std::istream_iterator<std::string>{}};

        std::vector<double> curr_log;
        for(auto s: curr_line){
            curr_log.push_back(std::stod(s));
        }
        load_feat1.push_back(Eigen::Vector2d(curr_log[0], curr_log[1]));
        load_feat2.push_back(Eigen::Vector2d(curr_log[2], curr_log[3]));
        load_norm1.push_back(Eigen::Vector2d(curr_log[4], curr_log[5]));
        load_norm2.push_back(Eigen::Vector2d(curr_log[6], curr_log[7]));

    }          
}

/**
 * @brief we test the difference between 8-point and 5-point methods
 * 1. from SIFT feature detected KITTI Odometry data
 * 2. from COLMAP's logged 2D features and normalized points
*/
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
    Image image1(argv[1], 1241, 376); // we assume they are kitti data
    Image image2(argv[2], 1241, 376);

    std::vector<sift::Keypoint> key_points1 = GetKeyPoints(image1);
    std::vector<sift::Keypoint> key_points2 = GetKeyPoints(image2);
    std::vector<std::pair<int, int>> matches = sift::find_keypoint_matches(key_points1, key_points2);

    // convert to feature corrdinates 
    std::vector<Eigen::Vector2d> points1 = SIFTPtsToVec(key_points1);
    std::vector<Eigen::Vector2d> points2 = SIFTPtsToVec(key_points2);

    std::cout << "check detected image features: " << std::endl;
    for(int i = 0; i < 5; i++) {
        std::cout << points1[i] << std::endl;
        std::cout << points2[i] << std::endl;
    }

    // start two view geometry estimation
    std::vector<Eigen::Vector2d> homo1;
    std::vector<Eigen::Vector2d> homo2;

    for(int i = 0; i < points1.size(); i++) {
        homo1.push_back(camera.ImageToWorld(points1[i]));
        homo2.push_back(camera.ImageToWorld(points2[i]));
    }

    std::cout << "check detected normalized points: " << std::endl;
    for(int i = 0; i < 5; i++) {
        std::cout << homo1[i] << std::endl;
        std::cout << homo2[i] << std::endl;
    }
    int start = 0;
    int end = 100;
    std::vector<Eigen::Vector2d> sl_homo1(homo1.begin() + start, homo1.begin() + end);
    std::vector<Eigen::Vector2d> sl_homo2(homo2.begin() + start, homo2.begin() + end);
    std::vector<Eigen::Vector2d> sl_pots1(points1.begin() + start, points1.begin() + end);
    std::vector<Eigen::Vector2d> sl_pots2(points2.begin() + start, points2.begin() + end);

    // essential matrix
    colmap::RANSACOptions E_options = colmap::RANSACOptions();
    E_options.max_error = 0.3;
    colmap::RANSAC<colmap::EssentialMatrixFivePointEstimator> e_ransac(E_options);
    const auto e_report = e_ransac.Estimate(sl_homo1, sl_homo2);
    Eigen::Matrix3d E = e_report.model;

    // fundamental matrix
    colmap::RANSACOptions F_options = colmap::RANSACOptions();
    F_options.max_error = 0.3;
    colmap::RANSAC<colmap::FundamentalMatrixSevenPointEstimator> f_ransac(F_options);
    std::cout << "before 7-points" << std::endl;
    const auto f_report = f_ransac.Estimate(sl_pots1, sl_pots2);
    Eigen::Matrix3d F = f_report.model;
    std::cout << "pass 7-points" << std::endl;

    // check results
    Eigen::Matrix3d k = camera.CalibrationMatrix();
    Eigen::Matrix3d k_t = k.transpose();

    std::cout << "check calibration and it's transpose" << std::endl;
    std::cout << k << std::endl;
    std::cout << k_t << std::endl;

    std::cout << "estimated essential: " << std::endl;
    std::cout << E << std::endl;
    std::cout << "estimated fundamental: " << std::endl;
    std::cout << F << std::endl;
    std::cout << "estimated converted fundamental: " << std::endl;
    std::cout << k_t*F*k << std::endl;

    double frob = frobeniusNormRot(E, k_t*F*k);
    std::cout << "frob between them is: " << frob << std::endl;

    // load colmap's log
    std::string log_path = "/tmp3/KITTI_Odometry/processed_colmap.txt";
    std::vector<Eigen::Vector2d> load_feat1;
    std::vector<Eigen::Vector2d> load_feat2;
    std::vector<Eigen::Vector2d> load_norm1; 
    std::vector<Eigen::Vector2d> load_norm2;

    LoadCOLMAPLog(log_path, load_feat1, load_feat2, load_norm1, load_norm2);
    std::cout << "check logged normalized points: " << std::endl;
    for(int i = 0; i < 5; i++) {
        std::cout << load_norm1[i] << std::endl;
        std::cout << load_norm2[i] << std::endl;
    }
    std::cout << "check logged feature points: " << std::endl;
    for(int i = 0; i < 5; i++) {
        std::cout << load_feat1[i] << std::endl;
        std::cout << load_feat2[i] << std::endl;
    }
    std::vector<Eigen::Vector2d> sl_feat1(load_feat1.begin() + start, load_feat1.begin() + end);
    std::vector<Eigen::Vector2d> sl_feat2(load_feat2.begin() + start, load_feat2.begin() + end);
    std::vector<Eigen::Vector2d> sl_norm1(load_norm1.begin() + start, load_norm1.begin() + end);
    std::vector<Eigen::Vector2d> sl_norm2(load_norm2.begin() + start, load_norm2.begin() + end);

    // essential matrix
    colmap::RANSAC<colmap::EssentialMatrixFivePointEstimator> elog_ransac(E_options);
    const auto elog_report = elog_ransac.Estimate(sl_norm1, sl_norm2);
    Eigen::Matrix3d E_log = elog_report.model;

    // fundamental matrix
    colmap::RANSAC<colmap::FundamentalMatrixSevenPointEstimator> flog_ransac(F_options);
    const auto flog_report = flog_ransac.Estimate(sl_feat1, sl_feat2);
    Eigen::Matrix3d F_log = flog_report.model;

    std::cout << "estimated essential from log data: " << std::endl;
    std::cout << E_log << std::endl;
    std::cout << "estimated fundamental from log data: " << std::endl;
    std::cout << F_log << std::endl;
    std::cout << "estimated converted fundamental from log: " << std::endl;
    std::cout << k_t*F_log*k << std::endl;

    double frob_log = frobeniusNormRot(E_log, k_t*F_log*k);
    std::cout << "frob between them is: " << frob_log << std::endl;

}