#include "base/essential_matrix.h"
#include "base/reconstruction.h"
#include "base/camera.h"
#include "base/pose.h"

#include "feature/sift.h"
#include "feature/image_sift.h"

#include "test_util.h"

#include "optim/ransac.h"
#include "estimators/pose.h"

//call EstimateRelativePose from estimators/pose
size_t GetNumInliers(const colmap::RANSACOptions& ransac_options,
                    Image& image1, Image& image2,
                    Eigen::Vector4d* qvec, Eigen::Vector3d* tvec){
    std::vector<MatchedVec> Matches;
    
    Matches = FindMatches(image1, image2);

    std::vector<Eigen::Vector2d> key_points1;
    std::vector<Eigen::Vector2d> key_points2;

    for (auto match : Matches) {
        key_points1.push_back(match.KeyPt1);
        key_points2.push_back(match.KeyPt2);
    }
    std::cout << "no segfault before call estimate " 
              << " num_of_matches " << Matches.size() << std::endl;
    size_t num_inliers = 
        colmap::EstimateRelativePose(ransac_options, key_points1, key_points2, 
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
    Image Image1(argv[1]);
    Image Image2(argv[2]);
    //get file path for ReadText
    std::string file_path = argv[3];
    //read files and instantiate class for each
    colmap::Reconstruction read_text = Reconstruction();
    read_text.ReadText(file_path);

    //hard coded w/ specify camera 1
    colmap::Camera Camera1 = read_text.Cameras().at(1);
    Eigen::Matrix3d Calibration = Camera1.CalibrationMatrix();

    colmap::RANSACOptions ransac_options = colmap::RANSACOptions();
    ransac_options.max_error = 0.1;

    //init q&t vec for the first two frames
    Eigen::Vector4d qvec1 = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d tvec1 = Eigen::Vector3d::Zero();
    Eigen::Vector4d qvec2 = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d tvec2 = Eigen::Vector3d::Zero();
    
    //Eigen::Matrix3x4d proj_mat1 = ProjMatFromQandT(qvec1, tvec1);

    size_t inliers = GetNumInliers(ransac_options,
                            Image1, Image2, &qvec2, &tvec2);

    //Eigen::Matrix3x4d proj_mat2 = ProjMatFromQandT(qvec2, tvec2);

    std::cout << inliers << std::endl;

    return 0;
}