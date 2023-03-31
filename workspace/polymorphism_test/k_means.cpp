#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <random>

using namespace std;

// function to calculate the euclidean distance between two vectors
double euclideanDistance(vector<double> a, vector<double> b) {
    double distance = 0.0;
    for (int i = 0; i < a.size(); i++) {
        distance += pow(a[i] - b[i], 2);
    }
    return sqrt(distance);
}

// function to assign each data point to the nearest centroid
void assignPointsToCentroids(vector<vector<double>>& data, vector<vector<double>>& centroids, vector<int>& assignments) {
    for (int i = 0; i < data.size(); i++) {
        double minDistance = numeric_limits<double>::max();
        int minIndex = -1;
        for (int j = 0; j < centroids.size(); j++) {
            double distance = euclideanDistance(data[i], centroids[j]);
            if (distance < minDistance) {
                minDistance = distance;
                minIndex = j;
            }
        }
        assignments[i] = minIndex;
    }
}

// function to update the centroids to the mean of the data points assigned to them
void updateCentroids(vector<vector<double>>& data, vector<vector<double>>& centroids, vector<int>& assignments) {
    for (int i = 0; i < centroids.size(); i++) {
        vector<double> sum(centroids[i].size(), 0.0);
        int count = 0;
        for (int j = 0; j < data.size(); j++) {
            if (assignments[j] == i) {
                for (int k = 0; k < sum.size(); k++) {
                    sum[k] += data[j][k];
                }
                count++;
            }
        }
        if (count > 0) {
            for (int k = 0; k < sum.size(); k++) {
                centroids[i][k] = sum[k] / count;
            }
        }
    }
}

// function to initialize the centroids randomly
void initializeCentroids(vector<vector<double>>& data, vector<vector<double>>& centroids, int k) {
    centroids.clear();
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, data.size() - 1);
    vector<int> indices;
    for (int i = 0; i < k; i++) {
        int index;
        do {
            index = dis(gen);
        } while (find(indices.begin(), indices.end(), index) != indices.end());
        indices.push_back(index);
        centroids.push_back(data[index]);
    }
}

// k-means algorithm
void kMeans(vector<vector<double>>& data, int k, int maxIterations) {
    vector<vector<double>> centroids;
    vector<int> assignments(data.size(), -1);
    initializeCentroids(data, centroids, k);
    int iterations = 0;
    while (iterations < maxIterations) {
        assignPointsToCentroids(data, centroids, assignments);
        updateCentroids(data, centroids, assignments);
        iterations++;
    }
    for (int i = 0; i < data.size(); i++) {
        cout << "Data point " << i << " is assigned to centroid " << assignments[i] << endl;
    }
}

int main() {
    vector<vector<double>> data = {{1.0, 1.0}, {1.5, 2.0}, {3.0, 4.0}, {5.0, 7.0}, {3.5, 5.0}, {4.5, 5.0}, {3.5, 4.5}};
    int k = 2; // number of clusters
    int maxIterations = 10; // maximum number of iterations for k-means
    kMeans(data, k, maxIterations);
    return 0;
}

