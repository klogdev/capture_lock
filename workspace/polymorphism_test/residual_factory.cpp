#include <ceres/ceres.h>

class ResidualCalculator{
    public:
        virtual bool Evaluate(double* residuals, double* parameters, double* jacobians) = 0;
};

class ReprojectionError : public ResidualCalculator{
    public:
        ReprojectionError(double obs_x, double obs_y):
            obs_x_(obs_x), obs_y_(obs_y){}
    
    virtual bool Evaluate(double* residuals, double* parameters, double* jacobians) override{
        
    }
}