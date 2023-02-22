#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>
#include <cmath>

#include "feature/image_sift.h"
#include "feature/sift.h"

#include "util/types.h"
#include "base/pose.h"

#include "test_util.h"

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

cv::Mat CentralCrop(cv::Mat orignal_image, int final_w, int final_h){
    int center_x = (orignal_image.cols)/2;
    int center_y = (orignal_image.rows)/2;

    //Rect(x_left,y_left,w,h)
    cv::Rect MyRoI(center_x-final_w/2, center_y-final_h/2,final_w,final_h);
    cv::Mat image_crop = orignal_image(MyRoI);

    return image_crop;
}

cv::Mat ResizeDown(cv::Mat orig_image, int final_w, int final_h){
    cv::Mat image;
    cv::resize(orig_image, image, cv::Size(final_w, final_h), cv::INTER_LINEAR);
    return image;
}


Eigen::Matrix3x4d ProjMatFromQandT(Eigen::Vector4d& qvec, Eigen::Vector3d& tvec){
    Eigen::Matrix3d rot_matrix = colmap::QuaternionToRotationMatrix(colmap::NormalizeQuaternion(qvec));
    std::cout << "(test) this is rotation matrix" << std::endl;
    std::cout << rot_matrix << std::endl;
    std::cout << "(test) this is t-vector " << tvec << std::endl;
    Eigen::Matrix3x4d proj_matrix;
    proj_matrix.block(0,0,3,3) = rot_matrix;
    proj_matrix.block(0,3,3,1) = tvec;

    std::cout << "(test) this is projection matrix" << std::endl;
    std::cout << proj_matrix << std::endl;
    
    return proj_matrix;
}

double QvecSquareErr(const Eigen::Vector4d qvec1, const Eigen::Vector4d qvec2){
    double error = 0.0;
    for (int i = 0; i < qvec1.size(); i++){
        error += std::abs(pow(qvec1(i) - qvec2(i), 2));
    }
    return error;
}

double TvecSquareErr(const Eigen::Vector3d tvec1, const Eigen::Vector3d tvec2){
    double error = 0.0;
    for (int i = 0; i < tvec1.size(); i++){
        error += std::abs(pow(tvec1(i) - tvec2(i), 2));
    }
    return error;
}