#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "test_util.h"

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need a image file path" << std::endl;
    }

    cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
    std::cout << "cv image shape " << image.cols << " " << image.rows << " "
              << image.channels() << std::endl;
    if(image.empty())
    {
        std::cout << "Could not read the image: " << argv[1] << std::endl;
        return 1;
    }
    
    cv::Mat crop_image = CentralCrop(image, 480, 640);

    std::cout << "cropped image shape " << crop_image.cols << " " 
                << crop_image.rows << " " << crop_image.channels() << std::endl;
    return 0;
}