#include <iostream> 
#include <string>

#include "image_sift.h"
#include "sift.h"

int main(int argc, char *argv[])
{
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    if (argc != 4) {
        std::cerr << "Usage: ./find_keypoints input.jpg (or .png)\n";
        return 0;
    }
    int final_w = std::stoi(argv[2]);
    int final_h = std::stoi(argv[3]);

    Image img(argv[1], final_w, final_h);
    img =  img.channels == 1 ? img : rgb_to_grayscale(img);

    std::vector<sift::Keypoint> kps = sift::find_keypoints_and_descriptors(img);
    Image result = sift::draw_keypoints(img, kps);
    result.save("result.jpg");

    std::cout << "Found " << kps.size() << " keypoints. Output image is saved as result.jpg\n";
    return 0;
}
