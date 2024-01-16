#include "kitti_odo_helper.h"
#include "file_reader/file_stream.h"
#include "file_reader/file_options.h"
#include "file_reader/data_types.h"
#include "calib_loader/calib_base.h"

#include <string>
#include <Eigen/Core>

int main(int argc, char** argv){
    if (argc < 2)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need to specify the dataset option" 
                  << std::endl;
    }

    std::string datasetName = argv[1];

    Dataset dataset = getDatasetFromName(datasetName);

    FileOptions files_to_run;
    InstantiateFiles(files_to_run, dataset);

    std::vector<std::vector<double>> cali_infos;
    IntrinsicFromKittiCali(files_to_run.calib_path, 
                           files_to_run.seq_num, 
                           cali_infos);
    // test the intrinsic matrix for a single sequence
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> intrinsic_test(cali_infos[0].data());
    std::cout << "current sequence's intrinsic matrix: " << std::endl;
    std::cout << intrinsic_test << std::endl;

    // test extrinsic matrix for a single frame
    std::vector<std::vector<double>> extrinsic_test;
    ExtrinsicFromKitti(files_to_run.pose_path, files_to_run.seq_num,
                       extrinsic_test);
    std::cout << "debug pose path: " << files_to_run.pose_path << std::endl;
    std::cout << "first frame's extrinsic matrix: " << std::endl;
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> extrinsic(extrinsic_test[0].data());
    std::cout << extrinsic << std::endl;

    // test relative motion of kitti via essential mat
    std::vector<std::string> image_stream;
    KITTIStream(image_stream, files_to_run.image_path,
                    files_to_run.start, files_to_run.end);

    

    return 0;
}