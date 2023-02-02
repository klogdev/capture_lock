#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "feature/image_sift.h"
#include "feature/sift.h"

struct MatchedVec
{
    /* data */
    Eigen::Vector2d KeyPt1;
    Eigen::Vector2d KeyPt2;
};

std::vector<MatchedVec> FindMatches(Image& Image1, Image& Image2);

cv::Mat CentralCrop(cv::Mat OriginalImage, int final_h, int final_w);