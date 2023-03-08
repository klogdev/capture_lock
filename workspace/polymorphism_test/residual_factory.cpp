#include <ceres/ceres.h>
#include <Eigen/Core>

class ResidualCalculator{
    public:
        virtual bool Evaluate(double* residuals, double* parameters, double* jacobians) = 0;
};

class ReprojectionError : public ResidualCalculator{
    public:
        ReprojectionError(Eigen::Vector2d point_2d, Eigen::Vector3d point_3d):
            obs_x_(point_2d(0)), obs_y_(point_2d(1)){}
    
    virtual bool Evaluate(std::vector<double>* residuals, std::vector<double>* parameters,
                        double* jacobians) override{
        //parameters should be a flattened vector with size 4x3
        Eigen::Vector3d project_3d;
        Eigen::Vector4d point3d_homo = Eigen::Vector4d::Identity();
        point3d_homo.topRows(3) = point_3d; 

        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> proj_mat(parameters.data(), 3, 4);
        project_3d = proj_mat*point3d_homo;


        residuals[0] = obs_x_ - project_3d(0)/project_3d(2);
        residuals[1] = obs_y_ - project_3d(1)/project_3d(2);

        //if jacobian != nullptr: update paras via ceres
    }

    private:
        double obs_x_;
        double obs_y_;
}

int main(){
    ceres::Problem problem;

}