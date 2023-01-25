#include <iostream> 
#include <string>

#include "feature/image_sift.h"
#include "feature/sift.h"

std::vector<std::pair<int, int>> FindMatches(Image& Image1, Image& Image2){
    //Image class is defined under feature/image_sift
    Image1_gray = Image1.channels == 1 ? Image1 : rgb_to_grayscale(Image1);
    Image2_gray = Image2.channels == 1 ? Image2 : rgb_to_grayscale(Image2);

    std::vector<sift::Keypoint> KeyPoints1 = sift::find_keypoints_and_descriptors(Image1_gray);
    std::vector<sift::Keypoint> KeyPoints2 = sift::find_keypoints_and_descriptors(Image2_gray);

    //here the int pairs are indices in KeyPoints vectors
    std::vector<std::pair<int, int>> Matches = sift::find_keypoint_matches(KeyPoints1, KeyPoints2);

    return Matches;
}