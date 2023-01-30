#include <Eigen/Core>

#include "base/essential_matrix.h"
#include "base/pose.h"
#include "estimators/pose.h"
#include "estimators/essential_matrix.h"
#include "feature/sift.h"
#include "feature/image_sift.h"
#include "feature_sift_test.h"

std::vector<Eigen::Matrix3d> EssentialMatrixFromFivePts(Image& Image1, Image& Image2){
    std::vector<MatchedVec> Matches;
    // TODO: This FindMatches() is defined in another file containing main()
    // function. Then it cannot be added into this module, because an executable
    // cannot target_link another executable. You can define some 
    // utils.h/utils.cpp for these functions and then use "add_library" in CMake
    // config to include them.
    // = FindMatches(image1, image2);

    std::vector<Eigen::Vector2d> KeyPoints1;
    std::vector<Eigen::Vector2d> KeyPoints2;

    for(auto match: Matches){
        KeyPoints1.push_back(match.KeyPt1);
        KeyPoints2.push_back(match.KeyPt2);
    }

    colmap::EssentialMatrixFivePointEstimator Estimator = colmap::EssentialMatrixFivePointEstimator();
    std::vector<Eigen::Matrix3d> CandidateMatrices = Estimator.Estimate(KeyPoints1, KeyPoints2);

    return CandidateMatrices;
}

int main(int argc, char** argv){
    if (argc < 3)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need two images file path" << std::endl;
    }

    //initialize the Image class by its path (feature/image_sift)
    Image Image1(argv[1]);
    Image Image2(argv[2]);

    std::vector<Eigen::Matrix3d> Essentials = EssentialMatrixFromFivePts(Image1, Image2);

    for(Eigen::Matrix3d M_t: Essentials){
        std::cout << "Here is the matrix m:\n" << M_t << std::endl;
    }
    return 0;
}