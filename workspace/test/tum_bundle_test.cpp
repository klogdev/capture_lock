#include "base/camera.h"
#include "base/camera_models.h"
#include "base/image.h"
#include "base/point3d.h"
#include "base/pose.h"

#include "ceres/ceres.h"
#include <iostream>
#include <unordered_map>

#include "file_reader/tum_rgbd.h"
#include "util_/math_helper.h"
#include "test/tum_rgbd_test.h"

// Define a simple TUMBundle function call for toy data.
void TestTUMBundle() {
    // Create a simple camera with arbitrary intrinsic parameters
    colmap::Camera camera;
    std::vector<double> intrinsic = {1, 0, 0}; // follow colmap's order f, cx, cy

    camera.SetCameraId(1); // only one camera
    camera.SetModelId(colmap::SimplePinholeCameraModel::model_id);
    camera.SetParams(intrinsic);

    // Define 8 box corners in camera space within a box of (±2, ±2, ±[4, 8]).
    std::vector<Eigen::Vector3d> box_corners = {
        {2, 2, 4}, {-2, 2, 4}, {2, -2, 4}, {-2, -2, 4},
        {2, 2, 8}, {-2, 2, 8}, {2, -2, 8}, {-2, -2, 8}
    };

    Eigen::Matrix3d rotation = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Vector3d translation(0.5, -0.5, 0.1);

    Eigen::Matrix3d rotation_2 = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Vector3d translation_2(1.5, -1.5, 0.5);

    // Create two images with arbitrary extrinsics (rotation and translation)
    std::unordered_map<int, colmap::Image*> global_img_map;
    auto* image0 = new colmap::Image();
    image0->SetImageId(0);
    image0->SetQvec(Eigen::Vector4d(1, 0, 0, 0)); // Identity quaternion
    image0->SetTvec(Eigen::Vector3d(0, 0, 0));     // Simple translation shift
    global_img_map[0] = image0;
    auto* image1 = new colmap::Image();
    image1->SetImageId(1);
    image1->SetQvec(colmap::RotationMatrixToQuaternion(rotation)); // Identity quaternion
    image1->SetTvec(translation);     // Simple translation shift
    global_img_map[1] = image1;
    auto* image2 = new colmap::Image();
    image2->SetImageId(2);
    image2->SetQvec(colmap::RotationMatrixToQuaternion(rotation_2)); // Identity quaternion
    image2->SetTvec(translation_2);     // Simple translation shift
    global_img_map[2] = image2;
    
    // Define a map for 3D points
    std::unordered_map<int, colmap::Point3D> global_3d_map;

    // Assign the 3D points to the 3D point map
    for (int i = 0; i <= 4; i++) {
        colmap::Point3D point3D;
        point3D.SetXYZ(Eigen::Vector3d(RandomGaussian(box_corners[i][0], 0.5), 
                                       RandomGaussian(box_corners[i][1], 0.5),
                                       RandomGaussian(box_corners[i][2], 0.5)));
        global_3d_map[i] = point3D;
    }

    // Assign feature observations to the images, with shared 3D points
    std::vector<Eigen::Vector2d> img0_2ds;
    for(int i = 0; i <= 3; i++) {
        Eigen::Vector2d curr_2d = Eigen::Vector2d(RandomGaussian(box_corners[i][0]/box_corners[i][2], 0.00), 
                                    RandomGaussian(box_corners[i][1]/box_corners[i][2], 0.00));
        img0_2ds.push_back(curr_2d);
        global_3d_map[i].Track().AddElement(0, i);
    }
    global_img_map[0]->SetPoints2D(img0_2ds);

    std::vector<Eigen::Vector2d> img1_2ds;
    for(int i = 0; i <= 3; i++) {
        Eigen::Vector3d curr_3d = rotation * box_corners[i] + translation;
        Eigen::Vector2d curr_2d = Eigen::Vector2d(RandomGaussian(curr_3d.x()/curr_3d.z(), 0.00), RandomGaussian(curr_3d.y()/curr_3d.z(), 0.00));
        img1_2ds.push_back(curr_2d);
        global_3d_map[i].Track().AddElement(1, i);
    }
    global_img_map[1]->SetPoints2D(img1_2ds);

    std::vector<Eigen::Vector2d> img2_2ds;
    for(int i = 0; i <= 3; i++) {
        Eigen::Vector3d curr_3d = rotation_2 * box_corners[i] + translation_2;
        Eigen::Vector2d curr_2d = Eigen::Vector2d(RandomGaussian(curr_3d.x()/curr_3d.z(), 0.00), RandomGaussian(curr_3d.y()/curr_3d.z(), 0.00));
        img2_2ds.push_back(curr_2d);
        global_3d_map[i].Track().AddElement(2, i);
    }
    global_img_map[2]->SetPoints2D(img2_2ds);

    // Set the 3D point IDs for the observations
    global_img_map[0]->SetPoint3DForPoint2D(0, 0); // Image 0, 2D point 0 -> 3D point 0
    global_img_map[0]->SetPoint3DForPoint2D(1, 1); // Image 0, 2D point 1 -> 3D point 1
    global_img_map[0]->SetPoint3DForPoint2D(2, 2); // Image 0, 2D point 2 -> 3D point 2
    global_img_map[0]->SetPoint3DForPoint2D(3, 3);

    // global_img_map[1]->SetPoint3DForPoint2D(0, 1); // Image 1, 2D point 0 -> 3D point 0
    // global_img_map[1]->SetPoint3DForPoint2D(1, 2); // Image 1, 2D point 1 -> 3D point 3
    // global_img_map[1]->SetPoint3DForPoint2D(2, 3); // Image 1, 2D point 2 -> 3D point 2
    // global_img_map[1]->SetPoint3DForPoint2D(3, 4);
    global_img_map[1]->SetPoint3DForPoint2D(0, 0); // Image 0, 2D point 0 -> 3D point 0
    global_img_map[1]->SetPoint3DForPoint2D(1, 1); // Image 0, 2D point 1 -> 3D point 1
    global_img_map[1]->SetPoint3DForPoint2D(2, 2); // Image 0, 2D point 2 -> 3D point 2
    global_img_map[1]->SetPoint3DForPoint2D(3, 3);

    global_img_map[2]->SetPoint3DForPoint2D(0, 0); // Image 0, 2D point 0 -> 3D point 0
    global_img_map[2]->SetPoint3DForPoint2D(1, 1); // Image 0, 2D point 1 -> 3D point 1
    global_img_map[2]->SetPoint3DForPoint2D(2, 2); // Image 0, 2D point 2 -> 3D point 2
    global_img_map[2]->SetPoint3DForPoint2D(3, 3);

    // CheckTUMResidual(global_img_map[0], camera, global_3d_map); 
    // CheckTUMResidual(global_img_map[1], camera, global_3d_map);

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

    std::vector<Eigen::Vector2d> point2ds;
    std::vector<Eigen::Vector3d> point3ds;

    RetrievePairsfromImage(global_img_map[0], global_3d_map, 
                           point2ds, point3ds);
    for(int i = 0; i < point3ds.size(); i++) {
        std::cout << "retrieve image 0's 3d point" << std::endl;
        std::cout << point3ds[i] << std::endl; 
    }
}

int main() {
    TestTUMBundle();
    return 0;
}
