#include <ceres/ceres.h>
#include <Eigen/Core>

enum class Residuals {Reprojection};

class ResidualCalculator{
    public:
     virtual bool operator()(const std::array<double>* parameters,
                           std::vector<double>* residuals) const = 0;
};

// class ReprojectionError : public ResidualCalculator{
//     public:
//      ReprojectionError(const Eigen::Vector2d& point_2d,
//                        const Eigen::Vector3d& point_3d)
//          : camera_point(point_2d), world_point(point_3d) {}

//      bool operator()(const std::array<double>* parameters, 
//                     std::vector<double>* residuals) const override {
//        // parameters should be a flattened vector with size 4x3
//        Eigen::Vector3d project_3d;
//        Eigen::Vector4d point3d_homo = Eigen::Vector4d::Identity();
//        point3d_homo.topRows(3) = world_point;

//        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
//                                       Eigen::RowMajor>>
//            proj_mat(parameters->data(), 3, 4);
//        project_3d = proj_mat * point3d_homo;

//        (*residuals)[0] = camera_point(0) - project_3d(0) / project_3d(2);
//        (*residuals)[1] = camera_point(1) - project_3d(1) / project_3d(2);

//        return true;
//      }

//     private:
//         Eigen::Vector2d camera_point;
//         Eigen::Vector3d world_point;
// };

class ReprojectionErrorFunctor {
 public:
  ReprojectionErrorFunctor(const std::array<double, 12>& some_param)
      : some_param_(some_param) {}

  template <typename T>
  bool operator()(const T* const x, const T* const y, T* e) const {
    // Just an example test, no practical meaning.
    e[0] = x[0] * y[0] + x[1] * y[1] + y[2];
    e[1] = some_param_[0] * x[0] * y[1] + x[1] * y[2];
    return true;
  }

 private:
  std::array<double, 12> some_param_;
};

int main(){
    ceres::Problem problem;
    Eigen::Vector2d camera_point(100.0, 120.0);
    Eigen::Vector3d triangulated_point(1.0, 2.0, 3.0);

    std::array<double, 12> some_param = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};


    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ReprojectionErrorFunctor, 2, 2, 3>(
            new ReprojectionErrorFunctor(some_param));
    problem.AddResidualBlock(cost_function, nullptr, camera_point.data(),
                             triangulated_point.data());

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