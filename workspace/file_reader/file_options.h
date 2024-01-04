#include <iostream>
#include <string>

#include "calib_base.h"

struct FileOptions{
    std::string dataset;
    int camera_model;
    float downsample;
};

struct COLMAPOptions{
    std::string sparse_cali;
    std::string image_seq;
};
