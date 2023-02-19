#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

#include <absl/random/random.h>
#include <absl/strings/str_format.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv){
  if (0) {
    cv::Mat image = cv::imread("/tmp2/images/P1180325.JPG");
    std::cout << "cv image shape " << image.cols << " " << image.rows << " "
              << image.channels() << std::endl;
    
    cv::Point center(100, 100);
    cv::Point dx(15, 0);
    cv::Point dy(0, 15);
    cv::Point line1_start = center - dx;
    cv::Point line1_end = center + dx;
    cv::Point line2_start = center - dy;
    cv::Point line2_end = center + dy;
    cv::line(image, line1_start, line1_end, cv::Scalar(0, 255, 0), 
            /*thickness=*/2, /*lineType=*/cv::LINE_AA);
    cv::line(image, line2_start, line2_end, cv::Scalar(0, 0, 255), 
            /*thickness=*/2, /*lineType=*/cv::LINE_AA);
    cv::imwrite("/tmp2/325_point.JPG", image);
  }

  {
    absl::BitGen bitgen;
    unsigned int index = absl::Uniform(bitgen, 0u, 10u);
    double fraction = absl::Uniform(bitgen, 0, 1.0);
    bool coin_flip = absl::Bernoulli(bitgen, 0.5);
  }

  {
    const std::string image_directory =
        "/tmp/datasets/colmap/south-building/images";
    for (int index = 150; index <= 340; ++index) {
      const std::string image_name = absl::StrFormat("P118%04d.JPG", index);
      const std::string image_path =
          absl::StrFormat("%s/%s", image_directory, image_name);
      cv::Mat image = cv::imread(image_path);
      std::cout << "image_path " << image_path << " " << (image.data != nullptr)
                << std::endl;
    }
  }

  return 0;
}