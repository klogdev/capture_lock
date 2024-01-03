#include "calib_base.h"
#include "base/camera.h"
#include "kitti_helper.h"

#include <string>

class KittiCalibReader: public CalibFileReader{
    public:
        KittiCalibReader(){};

        colmap::Camera GetIntrinsicMat(const std::string base_path, const std::string seq_num, 
                                       const double downscale) const override{
            std::vector<std::vector<double>> cali_info = IntrinsicFromKittiCali(calib_path,seq_num);

            colmap::Camera new_camera;
            new_camera.SetCameraId(1); // camera type 1 by defult
            new_camera.SetParams(cali_info[0]);
            new_camera.SetModelId(colmap::SimpleRadialCameraModel::model_id);
            new_camera.Rescale(downscale);

            return new_camera;
        }
}