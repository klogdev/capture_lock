#include <Eigen/Core>

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