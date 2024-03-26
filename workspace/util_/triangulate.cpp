#include <Eigen/Core>

#include "base/triangulation.h"
#include "base/projection.h"
#include "base/image.h"
#include "base/camera.h"

void TriangulateImage(const colmap::Image& frame1, const colmap::Image& frame2,
                      const colmap::Camera& camera,
                      const std::vector<Eigen::Vector2d>& match_vec1,
                      const std::vector<Eigen::Vector2d>& match_vec2,
                      std::vector<Eigen::Vector3d>& triangulated) {

    for(int i = 0; i < match_vec1.size(); i++) {
        Eigen::Vector2d normed_vec1 = camera.ImageToWorld(match_vec1[i]);
        Eigen::Vector2d normed_vec2 = camera.ImageToWorld(match_vec2[i]);

        // here the ProjectionMatrix is the Extrinsic 
        Eigen::Vector3d curr_tri = colmap::TriangulatePoint(frame1.ProjectionMatrix(),
                                                            frame2.ProjectionMatrix(),
                                                            normed_vec1, normed_vec2);

        triangulated.push_back(curr_tri);
    }
}

bool CheckTriangulateQuality(const Eigen::Matrix3x4d& proj_mat1, 
                             const Eigen::Matrix3x4d& proj_mat2,
                             const Eigen::Vector3d& proj_center1, 
                             const Eigen::Vector3d& proj_center2,
                             Eigen::Vector3d& xyz, double min_ang){
    
    bool check = colmap::HasPointPositiveDepth(proj_mat1, xyz) &&
                 colmap::HasPointPositiveDepth(proj_mat2, xyz) &&
                 colmap::CalculateTriangulationAngle(proj_center1,
                                                     proj_center2,
                                                     xyz) >= min_ang;

    return check;
}