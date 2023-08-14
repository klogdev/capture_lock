#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <cstdlib>

struct Point {
    double x, y;
    int cluster; // Which cluster this point belongs to

    Point(double x = 0, double y = 0, int cluster = -1): x(x), y(y), cluster(cluster) {}
};

double distance(const Point& a, const Point& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

std::vector<Point> kMeans(std::vector<Point>& data, int k, int maxIterations) {
    std::vector<Point> centroids(k);

    // Randomly initialize centroids
    for (int i = 0; i < k; ++i) {
        centroids[i] = data[std::rand() % data.size()];
    }

    int iterations = 0;
    bool changed = true; // extra flag to signal early stop

    while (changed && iterations < maxIterations) {
        changed = false;

        // Assign each data point to the closest centroid
        for (Point& point : data) {
            double minDist = std::numeric_limits<double>::max();
            int minIdx = 0;

            for (int i = 0; i < k; ++i) {
                double dist = distance(point, centroids[i]);
                if (dist < minDist) {
                    minDist = dist;
                    minIdx = i;
                }
            }

            if (point.cluster != minIdx) {
                point.cluster = minIdx;
                changed = true;
            }
        }

        // Recalculate the centroids
        std::vector<int> clusterSizes(k, 0);
        std::vector<Point> newCentroids(k, {0, 0});

        for (const Point& point : data) {
            newCentroids[point.cluster].x += point.x;
            newCentroids[point.cluster].y += point.y;
            clusterSizes[point.cluster]++;
        }

        for (int i = 0; i < k; ++i) {
            if (clusterSizes[i] > 0) {
                newCentroids[i].x /= clusterSizes[i];
                newCentroids[i].y /= clusterSizes[i];
            }
        }

        centroids = newCentroids;
        iterations++;
    }

    return centroids;
}

std::vector<Point> kMeansPlusPlusInitialization(const std::vector<Point>& data, int k) {
    std::vector<Point> centroids;
    std::vector<double> distances(data.size(), 0.0);

    // Step 1: Choose one centroid uniformly at random from the data points
    centroids.push_back(data[std::rand() % data.size()]);

    for (int i = 1; i < k; ++i) {
        double sum = 0;

        // Step 2: For each data point, compute D(x)
        for (size_t j = 0; j < data.size(); ++j) {
            distances[j] = distance(data[j], centroids[0]);
            for (size_t c = 1; c < i; ++c) {
                double tempDistance = distance(data[j], centroids[c]);
                if (tempDistance < distances[j]) {
                    distances[j] = tempDistance;
                }
            }
            sum += distances[j] * distances[j];
        }

        // Step 3: Choose next centroid
        double rnd = std::rand() / static_cast<double>(RAND_MAX) * sum;
        for (size_t j = 0; j < data.size(); ++j) {
            if ((rnd -= distances[j] * distances[j]) <= 0) {
                centroids.push_back(data[j]);
                break;
            }
        }
    }

    return centroids;
}

int main() {
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // Sample data points
    std::vector<Point> data = {
        {2, 3}, {5, 6}, {8, 9},
        {3, 2}, {6, 5}, {7, 8},
        {12, 15}, {14, 13}, {15, 14}
    };

    int k = 3; // number of clusters
    int maxIterations = 100;

    std::vector<Point> centroids = kMeans(data, k, maxIterations);

    for (int i = 0; i < k; ++i) {
        std::cout << "Centroid " << i << ": (" << centroids[i].x << ", " << centroids[i].y << ")\n";
    }

    return 0;
}
