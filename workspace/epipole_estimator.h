#include <Eigen/Core>
#include <vector>
#include <sift>
#include 

#include "optim/ransac.h"

class EpipoleEstimator{
    public: 
        Eigen::Vector3d Estimate(const Eigen::Vector2d& point_2d_01,
                                 const Eigen::Vector2d& point_2d_02,
                                 const Eigen::Vector2d& point_2d_11,
                                 const Eigen::Vector2d& point_2d_12);
        
        void Residual(std::vector<Eigen::Vector2d>& point1, 
                      std::vector<Eigen::Vector2d>& points2,
                      EigenVector3d& model, 
                      std::vector<double>* residuals);
        
};

//helper functions
Eigen::Vector3d Point2dToHomo(Eigen::Vector2d point_2d);

Eigen::Vector3d PointToLine(Eigen::Vector3d pixel1, Eigen::Vector3d pixel2);

Eigen::Vector3d LineToPoint(Eigen::Vector3d line1, Eigen::Vector3d line2);

double SquareError(Eigen::Vector3d homo1, Eigen::Vector3d homo2);