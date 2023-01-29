#include "base/essential_matrix.h"
#include "base/pose.h"
#include "feature/sift.h"
#include "feature/image_sift.h"

#include "estimators/pose.h"

//call EstimateRelativePose from estimators/pose
size_t GetNumInliers(const RANSACOptions& ransac_options,
                    Image image1, Image image2,
                    Eigen::Vector4d* qvec, Eigen::Vector3d* tvec){
    std::vector<MatchedVec>& Matches = FindMatches(image1, image2);

    std::vector<Eigen::Vector2d> key_points1;
    std::vector<Eigen::Vector2d> key_points2;

    for(auto match: Matches){
        key_points1.push_back(match.KeyPt1);
        key_points2.push_back(match.KeyPt2);
    }

    size_t num_inliers = EstimateRelativePose(ransac_options, key_points1,
                                            key_points2, qvec, tvec);
    return num_inliers;
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
    RANSACOptions& ransac_options;
    Eigen::Vector4d* qvec;
    Eigen::Vector3d* tvec;

    size_t inliers = GetNumInliers(ransac_options,
                            Image1, Image2, qvec, tvec);
    std::cout << inliers << std::endl;

    return 0;
}