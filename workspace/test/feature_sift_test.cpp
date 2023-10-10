#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

#include "feature/image_sift.h"
#include "feature/sift.h"
#include "test_util.h"


int main(int argc, char** argv){
    if (argc < 6)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need two images file path and the output path, also dim of image" << std::endl;
    }

    //initialize the Image class by its path (from feature/image_sift)
    Image image1(argv[1],576,768);
    Image image2(argv[2],576,768);

    int h = std::stoi(argv[4]);
    int w = std::stoi(argv[5]);

    //get cv mat separately for plot
    cv::Mat image_cv1 = ResizeDown(cv::imread(argv[1]),w,h);
    cv::Mat image_cv2 = ResizeDown(cv::imread(argv[2]),w,h);
    
    std::string img_path1 = argv[1];
    std::string img_path2 = argv[2];
    std::string file_path = argv[3];

    
    std::vector<MatchedVec> test_matches = FindMatches(image1, image2);

    std::cout << "number of matches: " << test_matches.size() << std::endl;

    // Concatenate images horizontally
    cv::Mat concat_img;
    cv::hconcat(image_cv1, image_cv2, concat_img);

    for (int i = 0; i < test_matches.size(); i++){
        cv::Point line1_start(test_matches[i].KeyPt1(0), test_matches[i].KeyPt1(1));
        cv::Point line1_end(test_matches[i].KeyPt2(0)+w, test_matches[i].KeyPt2(1));
        if(test_matches[i].KeyPt2(1) > 768)
            std::cout << "debug out of view" << std::endl;
        cv::line(concat_img, line1_start, line1_end, cv::Scalar(255, 0, 0), 
        /*thickness=*/2, /*lineType=*/cv::LINE_AA);  
    }

    cv::Point dummy_start(384,288);
    cv::Point dummy_end(1152,288);
    cv::line(concat_img, dummy_start, dummy_end, cv::Scalar(0, 255, 0), 
        /*thickness=*/2, /*lineType=*/cv::LINE_AA);  


    std::string img1_idx = img_path1.substr(18, 3); 
    std::string img2_idx = img_path2.substr(18, 3);    
    cv::imwrite("/tmp2/"+img1_idx+'_'+img2_idx+".JPG", concat_img);

    std::ofstream file(file_path, std::ios::trunc);
    file << "tested output of the matched points" << std::endl;

    for(MatchedVec match: test_matches){
        std::ostringstream line;
        line.precision(17);

        std::string line_string;
        line << match.KeyPt1[0] << ", " << match.KeyPt1[1] << std::endl;
        line << match.KeyPt2[0] << ", " << match.KeyPt1[1] << std::endl;

    }

    return 0;
}