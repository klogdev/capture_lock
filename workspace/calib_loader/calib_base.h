#ifndef CALIB_FILE_READER_H  
#define CALIB_FILE_READER_H

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <string>
#include <memory>

#include "base/camera.h"

#include "file_reader/data_types.h"

/**
 * @brief base class to loading the calibration data
 * for different dataset
*/
class CalibFileReader{
    public:
        virtual colmap::Camera GetIntrinsicMat(const std::string base_path, const std::string seq_num,
                                               double downscale,
                                               int width, int height) const = 0;
        static std::unique_ptr<CalibFileReader> CalibFileCreate(const Dataset file_type);
    
        virtual ~CalibFileReader() = default;        
};

#endif  // CALIB_FILE_READER_H  
