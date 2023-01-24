#include <iostream> 
#include <string>

#include "image_sift.h"
#include "sift.h"

std::vector<std::pair<int, int>> FindMatches(Image& a, Image& b){
    a = a.channels == 1 ? a : rgb_to_grayscale(a);
    b = b.channels == 1 ? b : rgb_to_grayscale(b);
}