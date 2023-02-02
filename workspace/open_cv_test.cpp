#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

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
    cv::imshow("Display window", image);
    int k = cv::waitKey(0); // Wait for a keystroke in the window
    if(k == 's')
    {
        cv::imwrite("test.png", image);
    }
    return 0;
}