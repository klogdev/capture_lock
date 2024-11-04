#include <ceres/iteration_callback.h>
#include <ceres/problem.h>

void PrintLargeResiduals(ceres::Problem& problem, double threshold = 1.0);

void PrintParameterBlocks(ceres::Problem& problem);