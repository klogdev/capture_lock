#include <iostream>
#include <vector>

using namespace std;

double linearRegression(vector<double>& x, vector<double>& y, double& slope, double& intercept, double alpha, int num_iters) {
    int m = x.size();
    double cost = 0;
    
    // Initialize the slope and intercept to 0
    slope = 0;
    intercept = 0;
    
    // Loop through the specified number of iterations
    for (int i = 0; i < num_iters; i++) {
        double slope_gradient = 0;
        double intercept_gradient = 0;
        double cost_sum = 0;
        
        // Calculate the gradient for each training example
        for (int j = 0; j < m; j++) {
            double y_hat = slope * x[j] + intercept;
            double error = y_hat - y[j];
            slope_gradient += error * x[j];
            intercept_gradient += error;
            cost_sum += error * error;
        }
        
        // Update the slope and intercept using the gradient and learning rate
        slope -= alpha * (1.0 / m) * slope_gradient;
        intercept -= alpha * (1.0 / m) * intercept_gradient;
        
        // Calculate the cost for the current iteration
        cost = (1.0 / (2 * m)) * cost_sum;
    }
    
    return cost;
}

int main() {
    // Example usage
    vector<double> x {1, 2, 3, 4, 5};
    vector<double> y {2, 4, 5, 4, 5};
    double slope, intercept;
    double cost = linearRegression(x, y, slope, intercept, 0.01, 1000);
    cout << "Slope: " << slope << endl;
    cout << "Intercept: " << intercept << endl;
    cout << "Cost: " << cost << endl;
    
    return 0;
}
