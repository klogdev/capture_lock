#include <Eigen/Core>
#include <vector>

#include "optim/ransac.h"

class EpipoleEstimator{
    public: 
        struct PointData{
            PointData(){}

            PointData(const Eigen::Vector2d& point_0, const Eigen::Vector2d& point_1)
                : point_lock(point_0), point_map(point_1) {}

        }
};