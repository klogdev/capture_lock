#ifndef ESTIMATE_SQPNP_H_
#define ESTIMATE_SQPNP_H_

#include <vector>
#include <Eigen/Core>

// implement SQPnP (via OpenCV)
class SQPnPEstimator {
    public:
        // The 2D image feature observations.
        typedef Eigen::Vector2d X_t;
        // The observed 3D features in the world frame.
        typedef Eigen::Vector3d Y_t;
        // The transformation from the world to the camera frame.
        typedef Eigen::Matrix3x4d M_t;

        // The minimum number of samples needed to estimate a model.
        static const int kMinNumSamples = 4;

        // Estimate the most probable solution of the PnP problem using DLS.
        //
        // @param points2D   Normalized 2D image points.
        // @param points3D   3D world points.
        //
        // @return           Most probable pose as a 3x4 matrix.
        static std::vector<M_t> Estimate(const std::vector<X_t>& points2D,
                                         const std::vector<Y_t>& points3D); 

        /**
         * @brief Calculate the squared reprojection error given a set of 2D-3D point
         * correspondences and a projection matrix.
        */
        static void Residuals(const std::vector<X_t>& points2D,
                              const std::vector<Y_t>& points3D,
                              const M_t& proj_matrix, std::vector<double>* residuals);

        /**
         * implement SQPnP's pose estimation by adapting interface from
         * https://github.com/terzakig/sqpnp.git
         */
        bool ComputeSQPnPPose(const std::vector<Eigen::Vector2d>& points2D,
                              const std::vector<Eigen::Vector3d>& points3D,
                              Eigen::Matrix3x4d* proj_matrix);
};

#endif  // ESTIMATE_SQPNP_H_
