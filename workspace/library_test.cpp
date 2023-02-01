#include <iostream> 
#include <string>
#include <fstream>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <absl/random/random.h>
#include <absl/random/random.h>

int main(int argc, char** argv){
  {
    cv::Mat image = cv::imread("/tmp/datasets/test/STanager-Shapiro-ML.jpg");
    std::cout << "cv image shape " << image.cols << " " << image.rows << " "
              << image.channels() << std::endl;
  }

  {
    absl::BitGen bitgen;
    unsigned int index = absl::Uniform(bitgen, 0u, 10u);
    double fraction = absl::Uniform(bitgen, 0, 1.0);
    bool coin_flip = absl::Bernoulli(bitgen, 0.5);
  }

  return 0;
}