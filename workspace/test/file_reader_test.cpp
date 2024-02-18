#include "file_reader/kitti_odo_helper.h"
#include "file_reader/colmap_pose.h"
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

    // 1. instantiate file options based on file type, 
    // @file_reader/file_option.h
    FileOptions files_to_run;
    InstantiateFiles(files_to_run, dataset);

    // 2. factory pattern to load calibration info
    // here we create a new instance Camera to absorb all cali info
    std::unique_ptr<CalibFileReader> file_reader = CalibFileReader::CalibFileCreate(dataset);
    colmap::Camera camera = file_reader->GetIntrinsicMat(files_to_run.calib_path,
                                                         files_to_run.seq_num,
                                                         files_to_run.downsample,
                                                         files_to_run.width,
                                                         files_to_run.height); 

    // read colmap's south building images of the first segment
    // or kitti's image with specified seq num
    std::vector<std::string> image_stream;
    if (dataset == Colmap) {
        COLMAPStream(image_stream, files_to_run.image_path, 
                        files_to_run.image_start, files_to_run.image_end);
    }
    else if (dataset == Kitti) {
        KITTIStream(image_stream, files_to_run.image_path,
                    files_to_run.image_start, files_to_run.image_end);
    }
    else if (dataset == KittiToColmap) {
        KITTIStream(image_stream, files_to_run.image_path,
                    files_to_run.image_start, files_to_run.image_end);
    }
    std::cout << "debug current data's image path is: " << 
    files_to_run.image_path << std::endl;

    // read g.t. poses, this request the user specifies the sequence num.
    // for colmap or colmap mapped kitti, we use "fake" seq num
    std::vector<std::vector<double>> extrinsic_gts;

    std::cout << "debug pose's path: " << files_to_run.pose_path << std::endl;

    if(dataset == Kitti)
        ExtrinsicFromKitti(files_to_run.pose_path, files_to_run.seq_num,
                           extrinsic_gts);
    else if(dataset == Colmap || dataset == KittiToColmap)
        ExtrinsicFromColmap(files_to_run.pose_path, extrinsic_gts,
                            files_to_run.colmap_opt.image_name_start, 
                            files_to_run.colmap_opt.image_name_end);

    std::cout << "first frame's extrinsic matrix element: " << std::endl;
    std::cout << extrinsic_gts[0][0] << std::endl;

}