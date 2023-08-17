#include <ceres/ceres.h>
#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/feature2d/feature2d.hpp>



class FeatureDetector{
    public:
        static FeatureDetector* create(std::string detect_type);
        virtual void detector(const cv::Mat* image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const = 0;

    private:
        virtual ~FeatureDetector() = default;

    protected:
        cv::Mat* getResize(const cv::Mat* orig, int h, int w) const{
            cv::Mat image;
            cv::resize(orig, image, cv::Size(w, h), cv::INTER_LINEAR);

            return image;
        }
    
};

class SIFTFeatureDetector: public FeatureDetector{
    public:
        SIFTFeatureDetector(){};
        void detector(cv::Mat* image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const override{
            //some detection
        }
};

class ORBFeatureDetector: public FeatureDetector{
    public:
     ORBFeatureDetector(){};
     void detector(const cv::Mat* image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const override{
        //detection
        cv::Ptr<cv::ORB> orb = cv::ORB::create();

        //preprocess, such as resize
        cv::Mat* img1 = getResize(image, 64, 64);

        // Detect ORB keypoints and compute descriptors
        orb->detectAndCompute(img1, cv::noArray(), keypoints, descriptors);

     }
};

//factory pattern
static FeatureDetector* FeatureDetector::create(std::string detector){
    switch(detector){
        case("sift"):
            return new SIFTFeatureDetector();
        case("orb"):
            return new ORBFeatureDetector();
        default:
            return nullptr;
    }
}

static std::map<std::string, std::pair<int,int>> parse(const std::string& s){
    std::vector<std::string> arr;
    std::istringstream iss(s);

    std::string token;

    while(std::getline(ss, token, ' ')){
        arr.push_back(token);
    }

    return {arr[0], std::make_pair((int)arr[1]), (int)arr[2]};
}