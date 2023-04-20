#include "calib_base.h"
#include "base/camera.h"
#include "base/reconstruction.h"

#include <string>

class ColmapCalibReader: public: CalibFileReader{
    public:
        ColmapCalibReader(){};

        colmap::Camera GetIntrinsicMat(const std::string base_path, const std::string seq_num, 
                                       const double downscale) const override{
            colmap::Reconstruction read_text = colmap::Reconstruction();
            read_text.ReadText(sparse_path);
            //assume we only have one camera
            colmap::Camera camera = read_text.Camera(1);//need to change focal, due to downsampling
            
            camera.SetModelId(colmap::SimpleRadialCameraModel::model_id);
            camera.Rescale(downscale);

            return camera;
        }
}
