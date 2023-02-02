#include "base/essential_matrix.h"
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
    std::cout << "no segfault before call estimate" << std::endl;
    size_t num_inliers = colmap::EstimateRelativePose(ransac_options, key_points1,
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
    colmap::RANSACOptions ransac_options = colmap::RANSACOptions();
    ransac_options.max_error = 0.05;

    Eigen::Vector4d* qvec(0);
    Eigen::Vector3d* tvec(0);

    size_t inliers = GetNumInliers(ransac_options,
                            Image1, Image2, qvec, tvec);
    std::cout << inliers << std::endl;

    return 0;
}