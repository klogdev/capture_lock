#include <ceres/ceres.h>
#include <Eigen/Core>

#include "estimate/least_sqr_pnp.h"

void LeastSquareSolver(const std::vector<Eigen::Vector2d>& points_2d, 
                       const std::vector<Eigen::Vector3d>& points_3d,
                       Eigen::Vector4d& quat_init, Eigen::Vector3d& trans_init,
                       int max_iters) {
    // Set up the problem
    ceres::Problem problem;
    for (size_t i = 0; i < points_2d.size(); i++) {
        // Assuming observations is a container of observed 2D points
        // and points3D is a container of corresponding 3D world points
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>(
                new ReprojectionError(points_2d[i], points_3d[i]));
        problem.AddResidualBlock(cost_function, nullptr, quat_init.data(), trans_init.data());
    }

    // Configure solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; // Or another suitable type for your problem
    options.minimizer_type = ceres::TRUST_REGION; // Trust region strategy typically uses GN or LM under the hood
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT; // Explicitly choose LM, which generalizes GN
    options.minimizer_progress_to_stdout = true;


    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;
}