#include <ceres/ceres.h>
#include <Eigen/Core>

class ResidualCalculator{
    public:
        virtual bool Evaluate(const std::vector<double>* parameters, std::vector<double>* residuals) const = 0;
};

class ReprojectionError : public ResidualCalculator{
    public:
        ReprojectionError(Eigen::Vector2d point_2d, Eigen::Vector3d point_3d):
                    camera_point(point_2d), world_point(point_3d){}
    
    virtual bool Evaluate(const std::vector<double>* parameters, std::vector<double>* residuals) const override{
        //parameters should be a flattened vector with size 4x3
        Eigen::Vector3d project_3d;
        Eigen::Vector4d point3d_homo = Eigen::Vector4d::Identity();
        point3d_homo.topRows(3) = world_point; 

        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> proj_mat(parameters->data(), 3, 4);
        project_3d = proj_mat*point3d_homo;


        (*residuals)[0] = camera_point(0) - project_3d(0)/project_3d(2);
        (*residuals)[1] = camera_point(1) - project_3d(1)/project_3d(2);

        return true;
    }

    private:
        Eigen::Vector2d camera_point;
        Eigen::Vector3d world_point;
};

int main(){
    ceres::Problem problem;
    Eigen::Vector2d camera_point(100.0, 120.0);
    Eigen::Vector3d triangulated_point(1.0, 2.0, 3.0);
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<ReprojectionError, 2, 12, 2, 3>(
          new ReprojectionError(camera_point, triangulated_point));

    problem.AddResidualBlock(cost_function, nullptr, new double[12],
                        camera_point.data(), triangulated_point.data());

    // Set the solver options
    ceres::Solver::Options options;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Print the results
    std::cout << "Optimization finished: " << summary.termination_type << std::endl;
    std::cout << "a = " << summary.FullReport() << std::endl;

    return 0;
}