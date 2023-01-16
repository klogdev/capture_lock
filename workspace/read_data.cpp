#include <fstream>
#include <string>
#include "read_data.h"
#include "util/string.h"
#include "base/point2d.h"
#include "base/point3d.h"

namespace colmap {

//initialize the class with file path/name
//to initialize a member public:
//1. Line(double len): length(len){}
//     double length;   
//2. Line(double len){
//       length = len;
//   }
//   double length;
ReadData::ReadData(std::string filePath): filename(filePath){
}

void ReadData::ReadImagesText(const std::string& path) {
  //clear the map for the new instance
  images_.clear(); 

  //a stream named as file
  std::ifstream file(path);
  CHECK(file.is_open()) << path;

  std::string line; //each line of the path file
  std::string item; // each item of current line (by string stream)

  //process the file line by line
  while (std::getline(file, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }
    //create a string stream to process current line, anaologous to .split()
    std::stringstream line_stream1(line);

    // ID
    std::getline(line_stream1, item, ' ');
    const uint32_t image_id = std::stoul(item);

    Image image; //default constructor
    image.SetImageId(image_id);

    image.SetRegistered(true);
    reg_image_ids_.push_back(image_id); //data member of ReadData

    // QVEC (qw, qx, qy, qz)
    std::getline(line_stream1, item, ' ');
    image.Qvec(0) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.Qvec(1) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.Qvec(2) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.Qvec(3) = std::stold(item);

    image.NormalizeQvec();

    // TVEC
    std::getline(line_stream1, item, ' ');
    image.Tvec(0) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.Tvec(1) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.Tvec(2) = std::stold(item);

    // CAMERA_ID
    std::getline(line_stream1, item, ' ');
    image.SetCameraId(std::stoul(item));

    // NAME
    std::getline(line_stream1, item, ' ');
    image.SetName(item);

    // POINTS2D
    if (!std::getline(file, line)) {
      break; //no key points for this image
    }

    StringTrim(&line);
    std::stringstream line_stream2(line);

    std::vector<Point2D> points2D_arr;
    std::vector<uint32_t> point3D_ids;

    if (!line.empty()) {
      while (!line_stream2.eof()) {  //not the end of file/line
        Point2D point; //new instance of point2d

        std::getline(line_stream2, item, ' ');
        // point.X() = std::stold(item);

        // std::getline(line_stream2, item, ' ');
        // point.Y() = std::stold(item);

        // points2D_arr.push_back(point);

        // std::getline(line_stream2, item, ' ');
        // if (item == "-1") {
        //   point3D_ids.push_back(kInvalidPoint3DId);
        // } else {
        //   point3D_ids.push_back(std::stoll(item)); //specify 3d's id
        //   point.point3D_id_ = std::stoll(item); //set the point3d's id for curr 2d point
        // }
      }
    }

    // image.SetUp(Camera(image.CameraId()));
    // image.SetPoints2D(points2D); //image's attr call point2d to set xy coord

    for (uint32_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
         ++point2D_idx) {
      if (point3D_ids[point2D_idx] != kInvalidPoint3DId) {
        image.SetPoint3DForPoint2D(point2D_idx, point3D_ids[point2D_idx]);
      }
    }

    images_.emplace(image.ImageId(), image);
  }
}


void ReadData::ReadCamerasText(const std::string& path) {
  cameras_.clear();

  std::ifstream file(path);
  CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  while (std::getline(file, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream(line);

    Camera camera;

    // ID
    std::getline(line_stream, item, ' ');
    camera.SetCameraId(std::stoul(item));

    // MODEL
    std::getline(line_stream, item, ' ');
    camera.SetModelIdFromName(item);

    // WIDTH
    std::getline(line_stream, item, ' ');
    camera.SetWidth(std::stoll(item));

    // HEIGHT
    std::getline(line_stream, item, ' ');
    camera.SetHeight(std::stoll(item));

    // PARAMS
    camera.Params().clear();
    while (!line_stream.eof()) {
      std::getline(line_stream, item, ' ');
      camera.Params().push_back(std::stold(item));
    }

    CHECK(camera.VerifyParams());

    cameras_.emplace(camera.CameraId(), camera);
  }
}

void ReadData::ReadPoints3DText(const std::string& path) {
  points3D_.clear();

  std::ifstream file(path);
  CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  while (std::getline(file, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream(line);

    // ID
    std::getline(line_stream, item, ' ');
    const uint32_t point3D_id = std::stoll(item);

    // Make sure, that we can add new 3D points after reading 3D points
    // without overwriting existing 3D points.
    num_added_points3D_ = std::max(num_added_points3D_, point3D_id);

    class Point3D point3D;

    // XYZ
    std::getline(line_stream, item, ' ');
    point3D.XYZ(0) = std::stold(item);

    std::getline(line_stream, item, ' ');
    point3D.XYZ(1) = std::stold(item);

    std::getline(line_stream, item, ' ');
    point3D.XYZ(2) = std::stold(item);

    // Color
    std::getline(line_stream, item, ' ');
    point3D.Color(0) = static_cast<uint8_t>(std::stoi(item));

    std::getline(line_stream, item, ' ');
    point3D.Color(1) = static_cast<uint8_t>(std::stoi(item));

    std::getline(line_stream, item, ' ');
    point3D.Color(2) = static_cast<uint8_t>(std::stoi(item));

    // ERROR
    std::getline(line_stream, item, ' ');
    point3D.SetError(std::stold(item));

    // // TRACK
    // while (!line_stream.eof()) {
    //   TrackElement track_el;

    //   std::getline(line_stream, item, ' ');
    //   StringTrim(&item);
    //   if (item.empty()) {
    //     break;
    //   }
    //   track_el.image_id = std::stoul(item);

    //   std::getline(line_stream, item, ' ');
    //   track_el.point2D_idx = std::stoul(item);

    //   point3D.Track().AddElement(track_el);
    // }

    //point3D.Track().Compress();

    points3D_.emplace(point3D_id, point3D);
  }
}

}  // namespace colmap