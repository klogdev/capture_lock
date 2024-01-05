#ifndef CALIB_FILE_READER_H  
#define CALIB_FILE_READER_H

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <string>

#include "base/camera.h"

/**
 * @brief base class to loading the calibration data
 * for different dataset
*/
class CalibFileReader{
    public:
        static CalibFileReader* Create(const std::string file_type);
        virtual colmap::Camera GetIntrinsicMat(const std::string base_path, const std::string seq_num,
                                               const double downscale) const = 0;

    protected:
        virtual ~CalibFileReader() = default;        
};
#endif  // CALIB_FILE_READER_H  
