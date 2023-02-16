#include "base/database.h"
#include "base/database_cache.h"
#include "sfm/incremental_mapper.h"
#include "sfm/incremental_triangulator.h"

#include "feature/image_sift.h"
#include "feature/sift.h"

Image LoadImage(const std::string folder, const std::string image_name){
    std::string abs_path = folder + image_name;
    Image image(abs_path);
    return image;
}