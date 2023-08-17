#include <random>
#include "optim/random_sampler.h"

struct RANSACOptions
{
    double max_error = 0.0;
    double min_inlier_ratio = 0.0;
    int max_iter = 10;
    int num_samples = 10;
    int num_trials = 100;
};

template<typename X_t, typename Y_t>
class Sampler{
    public:
        Sampler(){}

        void SampleXY(const std::vector<X_t>& X, const std::vector<Y_t>& Y, 
                      std::vector<X_t>& X_rand, std::vector<Y_t>& Y_rand){

                        int sample_size = X_rand.size();
                        unordered_set<int> seen;

                        int cnt = 0;
                        while(cnt < sample_size){
                            int trial = std::rand()%sample_size;
                            if(seen.find(trial) != seen.end())
                                continue;

                            seen.insert(trial);
                            X_rand.push_back(X[trial]);
                            Y_rand.push_back(Y[trial]);
                            cnt++;
                        }

                    }
};

// Estimator should have its own typedef, i.e. for X_t et.al
// also need encapsulate method such as Estimate/Residuals
template<typename Estimator, typename Sampler>
class RANSAC{
    public:
        RANSAC<typename Estimator, typename Sampler>::RANSAC(const RANSACOptions& options)
            :options_(options), sampler(Sampler()){
            // preprocessing, such as computer number of inliers needed
        }

        template<typename X_t, typename Y_t>
        bool Estimate(std::vector<X_t>& x, std::vector<Y_t>& y){
            std::vector<X_t> x_rand;
            std::vector<Y_t> y_rand;

            // in real implementation, we have separate logic to compare model
            // which count the # of inliers of the current model in addition to res
            double res = std::numeric_limits<double>::max();
            typename Estimator::M_t best_model;

            for(int i = 0; i < options_.num_trials; i++){
                sampler.SampleXY(x, y, x_rand, y_rand);
                typename Estimator::M_t model = estimator.Estimate(x_rand, y_rand);

                double current_residual = estimator.Residual(model, x, y);

                if (current_residual < best_residual) {
                    best_residual = current_residual;
                    best_model = model;
                }
            }

            // Assuming a threshold for a "good" model
            return best_residual <= options_.residual_threshold;
        }

    protected:
        RANSACOptions options_;
        Sampler sampler;
        Estimator estimator;
};

// the instance will be initialized as RANSAC<XxEstimator> ransac(options);