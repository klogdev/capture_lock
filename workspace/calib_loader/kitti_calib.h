#ifndef KITTI_CALIB_H_
#define KITTI_CALIB_H_

#include "calib_base.h"
#include "base/camera.h"
#include "base/camera_models.h"

#include "file_reader/kitti_odo_helper.h"

#include <string>

class KittiCalibReader: public CalibFileReader{
    public:
        KittiCalibReader(){};

        colmap::Camera GetIntrinsicMat(const std::string base_path, const std::string seq_num, 
                                       double downscale,
                                       int width, int height) const override{
            std::vector<std::vector<double>> cali_infos;
            // call customized file parser (from file_reader)
            IntrinsicFromKittiCali(base_path, seq_num, cali_infos);

            std::vector<double> pinhole_params;
            // manually get camera 0
            GetCameraModelParams(cali_infos[0], pinhole_params);

            colmap::Camera new_camera;
            new_camera.SetHeight(height);
            new_camera.SetWidth(width);
            new_camera.SetCameraId(1); // only one camera by defult
            new_camera.SetParams(pinhole_params);
            new_camera.SetModelId(colmap::SimplePinholeCameraModel::model_id);
            new_camera.Rescale(downscale);

            return new_camera;
        }
};

#endif  // KITTI_CALIB_H_