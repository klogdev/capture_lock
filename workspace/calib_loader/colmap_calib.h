#ifndef COLMAP_CALIB_H_
#define COLMAP_CALIB_H_

#include "calib_base.h"
#include "base/camera.h"
#include "base/camera_models.h"
#include "base/reconstruction.h"

#include <string>

class ColmapCalibReader: public CalibFileReader{
    public:
        ColmapCalibReader(){};

        colmap::Camera GetIntrinsicMat(const std::string base_path, const std::string seq_num, 
                                       double downscale,
                                       int width, int height) const override {
            colmap::Reconstruction read_text = colmap::Reconstruction();

            // this class method will call ReadCameraText to process the 
            // camera info from the sparse.txt file
            read_text.ReadText(base_path);
            // assume we only have one camera
            colmap::Camera camera = read_text.Camera(1); // need to change focal, due to downsampling

            camera.SetModelId(colmap::SimpleRadialCameraModel::model_id);

            camera.Rescale(downscale);

            return camera;
        }
};

#endif  // COLMAP_CALIB_H_