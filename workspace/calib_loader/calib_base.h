#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <string>

#include "base/camera.h"

//base class of loading calibration data
class CalibFileReader{
    public:
        static CalibFileReader Create(const std::string file_type);
        virtual colmap::Camera GetIntrinsicMat(const std::string base_path, const std::string seq_num,
                                               const double downscale) const = 0;

    private:
        virtual ~CalibFileReader() = default;        
}