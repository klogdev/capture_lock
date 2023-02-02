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
    Image Image1(argv[1]);
    Image Image2(argv[2]);
    
    std::string file_path = argv[3];
    
    std::vector<MatchedVec> TestMatches = FindMatches(Image1, Image2);

    std::ofstream file(file_path, std::ios::trunc);
    file << "tested output of the matched points" << std::endl;

    for(MatchedVec match: TestMatches){
        std::ostringstream line;
        line.precision(17);

        std::string line_string;
        line << match.KeyPt1[0] << ", " << match.KeyPt1[1] << std::endl;
        line << match.KeyPt2[0] << ", " << match.KeyPt1[1] << std::endl;

    }

    return 0;
}