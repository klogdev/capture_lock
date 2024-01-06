#include <string>
#include <memory>

#include "file_reader/data_types.h"

#include "calib_base.h"
#include "kitti_calib.h"
#include "colmap_calib.h"

// factory pattern
std::unique_ptr<CalibFileReader> 
CalibFileReader::CalibFileCreate(const Dataset file_type) {
    switch(file_type){
        case(Kitti):
            return std::make_unique<KittiCalibReader>(KittiCalibReader());
        case(Colmap): {
            // std::unique_ptr<CalibFileReader> reader;
            // reader.reset(new ColmapCalibReader());
            // return reader; 
            return std::make_unique<ColmapCalibReader>(ColmapCalibReader());
        }
        default:
            throw std::runtime_error("Unknown dataset");
    }
    return nullptr;
}