#include <ceres/iteration_callback.h>
#include <ceres/problem.h>

// Function to check and print residuals before/after optimization
void PrintLargeResiduals(ceres::Problem& problem, double threshold = 1.0) {
    std::vector<double> residuals;
    ceres::Problem::EvaluateOptions options;
    problem.Evaluate(options, nullptr, &residuals, nullptr, nullptr);

    std::cout << "Residuals larger than " << threshold << ":" << std::endl;
    for (size_t i = 0; i < residuals.size(); i += 2) {
        double magnitude = std::sqrt(residuals[i] * residuals[i] + residuals[i + 1] * residuals[i + 1]);
        if (magnitude > threshold) {
            std::cout << "Residual Block " << i / 2 << " Magnitude: " << magnitude 
                      << " (Residuals: [" << residuals[i] << ", " << residuals[i + 1] << "])" << std::endl;
        }
    }
}

void PrintParameterBlocks(ceres::Problem& problem) {
    // Retrieve all parameter blocks from the problem
    std::vector<double*> parameter_blocks;
    problem.GetParameterBlocks(&parameter_blocks);

    std::cout << "Total Parameter Blocks: " << parameter_blocks.size() << std::endl;

    for (size_t i = 0; i < parameter_blocks.size(); ++i) {
        int block_size = problem.ParameterBlockSize(parameter_blocks[i]);
        std::cout << "Parameter Block " << i << " (size " << block_size << "): ";
        
        // Print each value in the parameter block
        for (int j = 0; j < block_size; ++j) {
            std::cout << parameter_blocks[i][j] << " ";
        }
        std::cout << std::endl;
    }
}
