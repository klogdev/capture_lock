#include "calib_base.h"
#include <string>

//factory pattern
static CalibFileReader CalibFileReader::Create(std::string file_type){
    switch(file_type){
        case("kitti"):
            return new KittiCalibReader();
        case("colmap"):
            return new ColmapCalibReader();
        default:
            return nullptr;
    }
}