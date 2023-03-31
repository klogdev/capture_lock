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
        colmap::RandomSampler sampler;
        int num_samples = sampler.NumSamples();

        template <typename X_t, typename Y_t>
        bool Estimate(std::vector<X_t>& X, std::vector<Y_t>& Y){
            colmap::CHECK_EQ(X.size(), Y.size());
            
            std::vector<X_t> X_rand(num_samples);
            std::vector<Y_t> Y_rand(num_samples);
            typename Estimator::M_t model;
            const size_t num_samples = X.size();
            sampler.SampleXY(X, Y, &X_rand, &Y_rand);

            std::vector<typename Estimator::M_t> sample_models;
            for(int num_trials = 0; num_trials < options_.max_trials; num_trials++){
                sample_models = estimator.Estimate(X_rand, Y_rand);
                
                // Evaluate the quality of each model.
                for (const auto& sample_model : sample_models) {
                    std::vector<size_t> inliers;
                    double error_sum = 0.0;
                    for (size_t i = 0; i < num_samples; ++i) {
                        if (estimator.Test(sample_model, X[i], Y[i], options_.max_error)) {
                            inliers.push_back(i);
                            error_sum += estimator.Error(sample_model, X[i], Y[i]);
                        }
                    }

                    const double inlier_ratio = static_cast<double>(inliers.size()) / static_cast<double>(num_samples);
                    if (inlier_ratio >= options_.min_inlier_ration) {
                        // The model is good enough.
                        model = sample_model;
                        return true;
                    }
                }
            }
            // RANSAC failed to find a model that satisfies the threshold.
            return false;
        }

    protected:
        RANSACOptions options_;
};
