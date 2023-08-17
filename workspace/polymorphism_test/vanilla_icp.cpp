#include <Eigen/Dense>
#include <vector>
#include <iostream>

// Define a point structure
struct Point {
    double x, y, z;
};

// Function to compute centroid of a point cloud
Eigen::Vector3d computeCentroid(const std::vector<Point>& cloud) {
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& p : cloud) {
        centroid += Eigen::Vector3d(p.x, p.y, p.z);
    }
    centroid /= cloud.size();
    return centroid;
}

// Function to find closest point in target for each point in source
std::vector<Point> findClosestPoints(const std::vector<Point>& source, const std::vector<Point>& target) {
    std::vector<Point> correspondences;
    for (const auto& p : source) {
        double minDist = std::numeric_limits<double>::max();
        Point closest;
        for (const auto& q : target) {
            double dist = std::sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y) + (p.z - q.z) * (p.z - q.z));
            if (dist < minDist) {
                minDist = dist;
                closest = q;
            }
        }
        correspondences.push_back(closest);
    }
    return correspondences;
}

// Function to compute the optimal transformation
Eigen::Affine3d computeTransformation(const std::vector<Point>& source, const std::vector<Point>& target) {
    Eigen::Vector3d srcCentroid = computeCentroid(source);
    Eigen::Vector3d tgtCentroid = computeCentroid(target);

    // Build the cross-covariance matrix
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < source.size(); ++i) {
        Eigen::Vector3d srcPt = Eigen::Vector3d(source[i].x, source[i].y, source[i].z) - srcCentroid;
        Eigen::Vector3d tgtPt = Eigen::Vector3d(target[i].x, target[i].y, target[i].z) - tgtCentroid;
        H += srcPt * tgtPt.transpose();
    }

    // Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * U.transpose();
    if (R.determinant() < 0) { // Handling reflection case
        V.col(2) *= -1; // we force the det to be 1, which consistent with the def of rot mat
        R = V * U.transpose();
    }

    Eigen::Vector3d t = tgtCentroid - R * srcCentroid;

    // Build affine transformation
    Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
    transformation.linear() = R;
    transformation.translation() = t;
    
    return transformation;
}

int main() {
    // Example source and target point clouds (You'd typically load these from files or sensors)
    std::vector<Point> source = { {0, 0, 0}, {1, 0, 0}, {0, 1, 0} };
    std::vector<Point> target = { {0.5, 0.5, 0}, {1.5, 0.5, 0}, {0.5, 1.5, 0} };

    const int max_iterations = 50;
    const double tolerance = 1e-6;
    Eigen::Affine3d prevTransform, currentTransform = Eigen::Affine3d::Identity();

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<Point> correspondences = findClosestPoints(source, target);
        Eigen::Affine3d T = computeTransformation(source, correspondences);
        for (Point& p : source) {  // Apply transformation
            Eigen::Vector3d transformed = T * Eigen::Vector3d(p.x, p.y, p.z);
            p.x = transformed(0);
            p.y = transformed(1);
            p.z = transformed(2);
        }
        prevTransform = currentTransform;
        currentTransform = T * currentTransform;

        // Check for convergence
        if ((currentTransform.matrix() - prevTransform.matrix()).norm() < tolerance) {
            break;
        }
    }
}

   
