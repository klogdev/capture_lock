#include "calib_base.h"
#include "base/camera.h"
#include "kitti_helper.h"

#include <string>

class KittiCalibReader: public CalibFileReader{
    public:
        KittiCalibReader(){};

        colmap::Camera GetIntrinsicMat(const std::string base_path, const std::string seq_num, 
                                       const double downscale,
                                       int width, int height) const override{
            std::vector<std::vector<double>> cali_infos;
            // call customized file parser (from file_reader)
            IntrinsicFromKittiCali(calib_path, seq_num, cali_infos);

            std::vector<double> pinhole_params;
            GetCameraModelParams(cali_infos, pinhole_params);

            colmap::Camera new_camera;
            new_camera.SetHeight(height);
            new_camera.SetWidth(width);
            new_camera.SetCameraId(1); // only one camera by defult
            new_camera.SetParams(pinhole_params);
            new_camera.SetModelId(colmap::SimplePinholeCameraModel.model_id);
            new_camera.Rescale(downscale);

            return new_camera;
        }
}