#include <ceres/ceres.h>
#include <Eigen/Core>

class FeatureDetector{
    public:
        static FeatureDetector* create(std::string detect_type);
        virtual vector<int> detector(cv::Mat* image) const = 0;
    private:
        virtual ~FeatureDetector() = default;
    
};

class SIFTFeatureDetector: public FeatureDetector{
    public:
        SIFTFeatureDetector(){};
        vector<int> detector(cv::Mat* image) const override{
            //some detection
            return vector<int>();
        }
};

class ORBFeatureDetector: public FeatureDetector{
    public:
     ORBFeatureDetector(){};
     vector<int> detector(cv::Mat* image) const override{
        //detection
        return vector<int>();
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