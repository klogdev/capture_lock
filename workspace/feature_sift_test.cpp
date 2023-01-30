#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

#include "feature/image_sift.h"
#include "feature/sift.h"
#include "feature_sift_test.h"


std::vector<MatchedVec> FindMatches(Image& Image1, Image& Image2){
    //Image class is defined under feature/image_sift
    
    const Image& Image1_gray = Image1.channels == 1 ? Image1 : rgb_to_grayscale(Image1);
    const Image& Image2_gray = Image2.channels == 1 ? Image2 : rgb_to_grayscale(Image2);

    std::vector<sift::Keypoint> KeyPoints1 = sift::find_keypoints_and_descriptors(Image1_gray);
    std::vector<sift::Keypoint> KeyPoints2 = sift::find_keypoints_and_descriptors(Image2_gray);

    //here the int pairs are indices in KeyPoints vectors
    std::vector<std::pair<int, int>> Matches = sift::find_keypoint_matches(KeyPoints1, KeyPoints2);

    std::vector<MatchedVec> MatchVectors;

    for(int i = 0; i < Matches.size(); i++){
        int KeyIdx1 = Matches[i].first;
        int KeyIdx2 = Matches[i].second;
        sift::Keypoint CurrKey1 = KeyPoints1[KeyIdx1];
        sift::Keypoint CurrKey2 = KeyPoints2[KeyIdx2];
        Eigen::Vector2d Pt1(CurrKey1.x, CurrKey1.y);
        Eigen::Vector2d Pt2(CurrKey2.x, CurrKey2.y);

        MatchedVec CurrMatch;
        CurrMatch.KeyPt1 = Pt1;
        CurrMatch.KeyPt2 = Pt2;
        MatchVectors.push_back(CurrMatch);        
    }

    return MatchVectors;
}

int main(int argc, char** argv){
    if (argc < 4)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need two images file path and the output path" << std::endl;
    }

    //initialize the Image class by its path (feature/image_sift)
    Image Image1(argv[1]);
    Image Image2(argv[2]);
    std::string file_path = argv[4];
    std::vector<MatchedVec> TestMatches = FindMatches(Image1, Image2);

    std::ofstream file(file_path, std::ios::trunc);
    file << "tested output of the matched points" << std::endl;

    for(MatchedVec match: TestMatches){
        std::ostringstream line;
        line.precision(17);

        std::string line_string;
        line << match.KeyPt1[0] << ", " << match.KeyPt1[1] << std::endl;
        line << match.KeyPt2[0] << ", " << match.KeyPt1[1] << std::endl;

    }

    return 0;
}