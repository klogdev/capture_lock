#include "base/camera.h"
#include "base/camera_models.h"
#include "base/image.h"
#include "base/point3d.h"
#include "ceres/ceres.h"
#include <iostream>
#include <unordered_map>

#include "file_reader/tum_rgbd.h"

// Define a simple TUMBundle function call for toy data.
void TestTUMBundle() {
    // Create a simple camera with arbitrary intrinsic parameters
    colmap::Camera camera;
    std::vector<double> intrinsic = {500, 320, 240}; // follow colmap's order f, cx, cy

    camera.SetCameraId(1); // only one camera
    camera.SetModelId(colmap::SimplePinholeCameraModel::model_id);
    camera.SetParams(intrinsic);

    // Create two images with arbitrary extrinsics (rotation and translation)
    std::unordered_map<int, colmap::Image*> global_img_map;
    for (int i = 0; i < 2; ++i) {
        auto* image = new colmap::Image();
        image->SetImageId(i);
        image->SetQvec(Eigen::Vector4d(1, 0, 0, 0)); // Identity quaternion
        image->SetTvec(Eigen::Vector3d(i, 0, 0));     // Simple translation shift
        global_img_map[i] = image;
    }

    // Define a map for 3D points
    std::unordered_map<int, colmap::Point3D> global_3d_map;

    // Create four 3D points, with two points being shared between both images
    std::vector<Eigen::Vector3d> points3D = {
        Eigen::Vector3d(1, 1, 5),   // Shared between image 0 and image 1
        Eigen::Vector3d(2, -1, 6),  // Unique to image 0
        Eigen::Vector3d(-1, 1, 5),  // Shared between image 0 and image 1
        Eigen::Vector3d(3, -1, 7)   // Unique to image 1
    };

    // Assign the 3D points to the 3D point map
    for (int i = 0; i < points3D.size(); ++i) {
        colmap::Point3D point3D;
        point3D.SetXYZ(points3D[i]);
        global_3d_map[i] = point3D;
    }

    // Assign feature observations to the images, with shared 3D points
    global_img_map[0]->SetPoints2D({
        Eigen::Vector2d(320, 240), // Maps to point3D_id = 0 (shared)
        Eigen::Vector2d(100, 100), // Maps to point3D_id = 1 (unique)
        Eigen::Vector2d(200, 150)  // Maps to point3D_id = 2 (shared)
    });
    global_img_map[1]->SetPoints2D({
        Eigen::Vector2d(320, 240), // Maps to point3D_id = 0 (shared)
        Eigen::Vector2d(400, 300), // Maps to point3D_id = 3 (unique)
        Eigen::Vector2d(200, 150)  // Maps to point3D_id = 2 (shared)
    });

    // Set the 3D point IDs for the observations
    global_img_map[0]->SetPoint3DForPoint2D(0, 0); // Image 0, 2D point 0 -> 3D point 0
    global_img_map[0]->SetPoint3DForPoint2D(1, 1); // Image 0, 2D point 1 -> 3D point 1
    global_img_map[0]->SetPoint3DForPoint2D(2, 2); // Image 0, 2D point 2 -> 3D point 2

    global_img_map[1]->SetPoint3DForPoint2D(0, 0); // Image 1, 2D point 0 -> 3D point 0
    global_img_map[1]->SetPoint3DForPoint2D(1, 3); // Image 1, 2D point 1 -> 3D point 3
    global_img_map[1]->SetPoint3DForPoint2D(2, 2); // Image 1, 2D point 2 -> 3D point 2

    // Print initial setup to verify correctness
    std::cout << "Initial 3D points:\n";
    for (const auto& [id, point3D] : global_3d_map) {
        std::cout << "3D point " << id << ": " << point3D.XYZ().transpose() << "\n";
    }

    // Call the TUMBundle function with the setup data
    TUMBundle(global_img_map, global_3d_map, camera);

    // Print results after bundle adjustment
    std::cout << "Adjusted 3D points:\n";
    for (const auto& [id, point3D] : global_3d_map) {
        std::cout << "3D point " << id << ": " << point3D.XYZ().transpose() << "\n";
    }
}

int main() {
    TestTUMBundle();
    return 0;
}
