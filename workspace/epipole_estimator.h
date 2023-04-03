#include <Eigen/Core>
#include <vector>
#include <sift>
#include 

#include "optim/ransac.h"

class EpipoleEstimator{
    public: 
        struct PixelPair
        {
            PixelPair(const Eigen::Vector2d& point_lock_, const Eigen::Vector2d& point_comp_)
                :point_lock(point_lock_), point_comp(point_comp_) {}
            Eigen::Vector2d point_lock;
            Eigen::Vector2d point_comp;
        };

        typedef PixelPair X_t;
        typedef PixelPair Y_t;
        typedef Eigen::Vector3d M_t;

        // The minimum number of samples needed to estimate a model.
        static const int kMinNumSamples = 1;
        
        //should be a size one vector
        Eigen::Vector3d Estimate(const std::vector<X_t>& pair1,
                                 const std::vector<Y_t>& pair2);
        
        void Residuals(const std::vector<X_t>& pairs, 
                       const std::vector<Y_t>& pairs2,
                       const M_t& model, 
                       std::vector<double>* residuals);
        
};

bool EstimateEpipole(const colmap::RANSACOptions& ransac_options,
                     const std::vector<PixelPair>& pairs1
                     const std::vector<PixelPair>& pairs2
                     std::vector<char>* inlier_mask, Eigen::Vector3d* model);

//helper functions
Eigen::Vector3d Point2dToHomo(Eigen::Vector2d point_2d);

Eigen::Vector3d PointToLine(Eigen::Vector3d pixel1, Eigen::Vector3d pixel2);

Eigen::Vector3d LineToPoint(Eigen::Vector3d line1, Eigen::Vector3d line2);

double SquareError(Eigen::Vector3d homo1, Eigen::Vector3d homo2);