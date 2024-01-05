#include "calib_base.h"
#include "kitti_calib.h"
#include "colmap_calib.h"

#include <string>

// factory pattern
static CalibFileReader* CalibFileReader::Create(std::string file_type){
    switch(file_type){
        case("kitti"):
            return new KittiCalibReader();
        case("colmap"):
            return new ColmapCalibReader();
        default:
            throw std::runtime_error("Unknown dataset");
    }
}