#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

#include "feature/image_sift.h"
#include "feature/sift.h"
#include "test_util.h"


int main(int argc, char** argv){
    if (argc < 4)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need two images file path and the output path" << std::endl;
    }

    //initialize the Image class by its path (feature/image_sift)
    Image image1(argv[1]);
    Image image2(argv[2]);
    //get cv mat separately for plot
    // cv::Mat image_cv1 = CentralCrop(cv::imread(argv[1]),800,600);
    // cv::Mat image_cv2 = CentralCrop(cv::imread(argv[2]),800,600);

    //get cv mat separately for plot
    cv::Mat image_cv1 = ResizeDown(cv::imread(argv[1]),800,600);
    cv::Mat image_cv2 = ResizeDown(cv::imread(argv[2]),800,600);
    
    std::string file_path = argv[3];
    
    std::vector<MatchedVec> test_matches = FindMatches(image1, image2);

    std::cout << "number of matches: " << test_matches.size() << std::endl;

    int num_plot_pts = 10;

    for (int i = 0; i < test_matches.size(); i++){
        cv::Point line1_start(test_matches[i].KeyPt1(0), test_matches[i].KeyPt1(1));
        cv::Point line1_end(test_matches[i].KeyPt2(0), test_matches[i].KeyPt2(1));
    
        cv::line(image_cv1, line1_start, line1_end, cv::Scalar(255, 0, 0), 
        /*thickness=*/2, /*lineType=*/cv::LINE_AA);
        cv::line(image_cv2, line1_start, line1_end, cv::Scalar(255, 0, 0), 
        /*thickness=*/2, /*lineType=*/cv::LINE_AA);

        cv::line(image_cv1, line1_start, line1_start, cv::Scalar(0, 0, 255), 
        /*thickness=*/2, /*lineType=*/cv::LINE_AA);
        cv::line(image_cv1, line1_end, line1_end, cv::Scalar(0, 255, 0), 
        /*thickness=*/2, /*lineType=*/cv::LINE_AA);

        cv::line(image_cv2, line1_start, line1_start, cv::Scalar(0, 0, 255), 
        /*thickness=*/2, /*lineType=*/cv::LINE_AA);
        cv::line(image_cv2, line1_end, line1_end, cv::Scalar(0, 255, 0), 
        /*thickness=*/2, /*lineType=*/cv::LINE_AA);
        
    }

    cv::imwrite("/tmp2/317_point.JPG", image_cv1);
    cv::imwrite("/tmp2/318_point.JPG", image_cv2);

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