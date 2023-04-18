#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <string>

#include "base/camera.h"

class CalibFileReader{
    public:
        static CalibFileReader Create(const std::string file_type);
        virtual colmap::Camera GetIntrinsicMat(const std::string file_path) const = 0;
    private:
        virtual ~CalibFileReader() = default;
}