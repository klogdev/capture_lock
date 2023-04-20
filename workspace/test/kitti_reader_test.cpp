#include "kitti_helper.h"
#include "file_stream.h"

#include <string>
#include <Eigen/Core>

int main(int argc, char** argv){
    if (argc < 3)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need both kitti clibration and gyroscope file path" 
                  << std::endl;
    }

    std::string seq_num = "00"; //default read the left gray camera

    std::string calib_path = argv[1];
    std::string gyro_path = argv[2];

    std::vector<std::vector<double>> cali_info = IntrinsicFromKittiCali(calib_path,seq_num);

    Eigen::Matrix3d K = ProjMatFromCali(cali_info,0);

}