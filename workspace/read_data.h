#ifndef READ_DATA_H_
#define READ_DATA_H_

#include "base/camera.h"
#include "base/image.h"
#include "base/point3d.h"
#include "util/types.h"
#include <unordered_map>
#include <vector>

namespace colmap {

class ReadData{
    public:

    // Read data from text or binary file. 
    ReadData(std::string filePath);
    std::string filename;
    void ReadCamerasText(const std::string& path);
    void ReadImagesText(const std::string& path);
    void ReadPoints3DText(const std::string& path);

    // accessors for the private members, callable
    const std::unordered_map<camera_t, class Camera> & Cameras() const;
    const std::unordered_map<image_t, class Image> & Images() const;
    const std::unordered_map<point3D_t, class Point3D> & Points3D() const;
    
    //map to store images/camera instances
    private:
    std::unordered_map<camera_t, class Camera> cameras_; //hashmap vs EigenMap?
    std::unordered_map<image_t, class Image> images_;
    std::unordered_map<point3D_t, class Point3D> points3D_;

    // { image_id, ... } where `images_.at(image_id).registered == true`.
    std::vector<image_t> reg_image_ids_;

    uint32_t num_added_points3D_ = 0;
};

}  // namespace colmap

#endif // READ_DATA_H_ 
