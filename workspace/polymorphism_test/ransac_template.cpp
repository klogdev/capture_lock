#include <random>
#include "optim/random_sampler.h"

struct RANSACOptions
{
    double max_error = 0.0;

    double min_inlier_ration = 0.0;
};

template <typename Estimator> //model for estimation
class RANSAC{
    public:
        RANSAC(const RANSACOptions& options){}

        template <typename X_t, typename Y_t>
        bool Estimate(std::vector<X_t>& X, std::vector<Y_t>& Y){
            colmap::CHECK_EQ(X.size(), Y.size());


        }
}