#include <iostream>
#include <fstream>
#include <unordered_map>
#include <Eigen/Core>
#include <string>

#include "base/image.h"

void SavePoseToTxt(std::string outfile_name, 
                   std::unordered_map<int,colmap::Image>& global_image_map) {
    

    // Open a text file for writing
    std::ofstream outfile(outfile_name);

    if (!outfile) {
        std::cerr << "Could not open the file!" << std::endl;
    }

    // Write the header
    outfile << "IMAGE_ID,QW,QX,QY,QZ,TX,TY,TZ\n";

    // Write the pose data
    for (const auto& image : global_image_map) {
        int image_id = image.first;
        colmap::Image curr_image = image.second;
        const Eigen::Vector4d& quaternion = curr_image.Qvec();
        const Eigen::Vector3d& translation = curr_image.Tvec();  

        outfile << image_id << " ";
        
        for (int i = 0; i < quaternion.size(); ++i) {
            outfile << quaternion[i] << " ";
        }
        
        for (int i = 0; i < translation.size(); ++i) {
            outfile << translation[i];
            if (i < translation.size() - 1) {
                outfile << " ";
            }
        }
        
        outfile << "\n";
    }

    // Close the file
    outfile.close();
}
