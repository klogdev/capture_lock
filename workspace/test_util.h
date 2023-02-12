#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "feature/image_sift.h"
#include "feature/sift.h"
#include "util/types.h"

struct MatchedVec
{
    /* data */
    Eigen::Vector2d KeyPt1;
    Eigen::Vector2d KeyPt2;
};

std::vector<MatchedVec> FindMatches(Image& Image1, Image& Image2);

cv::Mat CentralCrop(cv::Mat orignal_image, int final_h, int final_w);

cv::Mat ResizeDown(cv::Mat original_image, int final_h, int final_w);

Eigen::Matrix3x4d ProjMatFromQandT(Eigen::Vector4d& qvec, Eigen::Vector3d& tvec);

//calculate reprojection error
double QvecSquareErr(const Eigen::Vector4d qvec1, const Eigen::Vector4d qvec2);

double TvecSquareErr(const Eigen::Vector3d tvec1, const Eigen::Vector3d tvec2);