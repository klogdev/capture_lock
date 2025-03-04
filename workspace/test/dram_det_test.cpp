#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>

#include "pnp/pnp_test_data.h"
#include "estimate/adj_quat.h"

void AnalyzePlanarEigenvalues() {
    PlanarPerturb generator;
    std::vector<std::vector<Eigen::Vector2d>> points2D;
    std::vector<std::vector<Eigen::Vector3d>> points3D;
    std::vector<Eigen::Matrix3x4d> composed_extrinsic;

    // Generate data
    generator.generate(points2D, points3D, composed_extrinsic);

    // For each perturbation level (sigma)
    int sample_count = 0;
    for (double sigma = PlanarPerturb::sigma_s; sigma <= PlanarPerturb::sigma_e; sigma += 0.02) {
        std::vector<double> ratios_for_sigma;
        std::vector<double> det_values_for_sigma;

        // Process all samples for this sigma
        for (int sample = 0; sample < 500; sample++) {
            int idx = sample_count * 500 + sample;
            
            std::vector<double> dets;
            double ratio;
            MakeDeterminants2D(points3D[idx], points2D[idx], dets, ratio);
            
            ratios_for_sigma.push_back(ratio);
            det_values_for_sigma.push_back(dets[0]); // mat1's determinant
        }

        // Calculate statistics for this sigma
        double mean_ratio = std::accumulate(ratios_for_sigma.begin(), 
                                          ratios_for_sigma.end(), 0.0) / ratios_for_sigma.size();

        // Sort for median and percentiles
        std::sort(ratios_for_sigma.begin(), ratios_for_sigma.end());
        double median_ratio = ratios_for_sigma[ratios_for_sigma.size() / 2];
        double percentile_5th = ratios_for_sigma[ratios_for_sigma.size() * 5 / 100];
        double percentile_95th = ratios_for_sigma[ratios_for_sigma.size() * 95 / 100];

        std::cout << std::fixed << std::setprecision(6)
                  << "Sigma: " << std::setw(6) << sigma 
                  << " | Ratio - Mean: " << std::setw(10) << mean_ratio 
                  << " | Median: " << std::setw(10) << median_ratio 
                  << " | 5th: " << std::setw(10) << percentile_5th
                  << " | 95th: " << std::setw(10) << percentile_95th 
                  << std::endl;

        sample_count++;
    }
}

int main() {
    AnalyzePlanarEigenvalues();
    return 0;
}
