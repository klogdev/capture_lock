#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdlib>

double monteCarloAreaEstimation(const cv::Mat& image, int numSamples) {
    int width = image.cols;
    int height = image.rows;

    int pointsInside = 0;

    for (int i = 0; i < numSamples; ++i) {
        int x = std::rand() % width;
        int y = std::rand() % height;

        if (image.at<uchar>(y, x) > 0) { // Assuming a grayscale image
            pointsInside++;
        }
    }

    // Calculate the fraction of sampled points that are inside cells and multiply by image's pixel count
    return (double(pointsInsideCells) / numSamples) * (width * height);
}


int main(int argc, char **argv) {
    // Read the image in grayscale
    if(argc < 2)
        std::cerr << "must specify the image path" << std::endl;

    std::string path = argv[1];
    cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Could not open or find the image!" << std::endl;
        return -1;
    }

    // Preprocessing if needed
    cv::Mat blurred;
    cv::GaussianBlur(image, blurred, cv::Size(5, 5), 0);

    // thresholding or binarization; here for BINARY_INV: > thre = 0, otherwise max valued
    cv::Mat thresh;
    cv::threshold(blurred, thresh, 127, 255, cv::THRESH_BINARY_INV);

    // via Suzuki algorithm, returns list of pixels as the contour
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // get Area by built-in fn by Green's theorem 
    double total_area = 0.0;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        total_area += area;
    }

    // visualization
    cv::Mat color_image;
    cv::cvtColor(image, color_image, cv::COLOR_GRAY2BGR);

    for (const auto& contour : contours) {
        cv::drawContours(color_image, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2); // Draw in green
    }

    cv::imshow("Detected Areas", color_image);
    cv::waitKey(0);

    // area via MC
    double estimatedArea = monteCarloAreaEstimation(image, 100000); // Using 100,000 samples for the image
    std::cout << "Estimated Area: " << estimatedArea << std::endl;
}