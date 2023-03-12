#include <random>
#include "optim/random_sampler.h"

struct RANSACOptions
{
    double max_error = 0.0;

    double min_inlier_ration = 0.0;

    int max_trials = 0;
};

template <typename Estimator> //model for estimation
class RANSAC{
    public:
        RANSAC(const RANSACOptions& options)
        :options_(options) {}

        Estimator estimator;

    template <typename X_t, typename Y_t>
    bool Estimate(std::vector<X_t>& X, std::vector<Y_t>& Y){
        colmap::CHECK_EQ(X.size(), Y.size());
        int num_samples = 10;
        
        std::vector<X_t> X_rand(num_samples);
        std::vector<Y_t> Y_rand(num_samples);
        const size_t num_samples = X.size();
        sampler.SampleXY(X, Y, &X_rand, &Y_rand);

        for(int num_trials; num_trials < options_.max_trials; num_trials++){
            estimator.Estimate(X_rand, Y_rand);
            

        }

    }

    protected:
        RANSACOptions options_;
}