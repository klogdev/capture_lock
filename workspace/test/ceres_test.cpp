#include <ceres/ceres.h>

// Define a functor that computes the residual (difference between observed and predicted values)
struct QuadraticResidual {
  QuadraticResidual(double x_observed, double f_observed)
      : x_observed_(x_observed), f_observed_(f_observed) {}

  template <typename T>
  bool operator()(const T* const parameters, T* residual) const {
    // Compute the predicted value of the function using the current parameter values
    const T a = parameters[0];
    const T b = parameters[1];
    const T c = parameters[2];
    const T x_predicted = static_cast<T>(x_observed_);
    const T f_predicted = a * x_predicted * x_predicted + b * x_predicted + c;

    // Compute the residual (difference between observed and predicted values)
    residual[0] = f_observed_ - f_predicted;

    return true;
  }

 private:
  const double x_observed_;
  const double f_observed_;
};

int main() {
  // Set up the problem
  ceres::Problem problem;

  // Add the measurements
  const double x_observed = 1.0;
  const double f_observed = 3.0;
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<QuadraticResidual, 1, 3>(
          new QuadraticResidual(x_observed, f_observed));
  problem.AddResidualBlock(cost_function, nullptr, new double[3]);

  // Set the solver options
  ceres::Solver::Options options;
  options.max_num_iterations = 100;

  // Run the solver
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Print the results
  std::cout << "Optimization finished: " << summary.termination_type << std::endl;
  std::cout << "a = " << summary.FullReport() << std::endl;

  return 0;
}
