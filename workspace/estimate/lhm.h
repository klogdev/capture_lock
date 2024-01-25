#ifndef ESTIMATE_LHM_H_
#define ESTIMATE_LHM_H_

#include <vector>

#include <Eigen/Core>

struct LHMOptions
{  
    // convergence tolerance of lhm
    double lhm_tolerance = 1e-5;

    // lower bound of objective fxn
    double lhm_epsilon = 1e-8;

    // max iteration for current estimation
    int lhm_iter = 35;
};


// Implementation of the LHM method
class LHMEstimator {
    public:
        // The 2D image feature observations.
        typedef Eigen::Vector2d X_t;
        // The observed 3D features in the world frame.
        typedef Eigen::Vector3d Y_t;
        // The transformation from the world to the camera frame.
        typedef Eigen::Matrix3x4d M_t;

        // The minimum number of samples needed to estimate a model.
        static const int kMinNumSamples = 3;

        // Estimate the most probable solution of the P3P problem from a set of
        // three 2D-3D point correspondences.
        //
        // @param points2D   Normalized 2D image points as 3x2 matrix.
        // @param points3D   3D world points as 3x3 matrix.
        //
        // @return           Most probable pose as length-1 vector of a 3x4 matrix.
        static std::vector<M_t> Estimate(const std::vector<X_t>& points2D,
                                         const std::vector<Y_t>& points3D);

    private:
        /**
         * @brief estimate the absolute pose via LHM from corresponded 
         * 2D, 3D points
        */
        bool ComputeLHMPose(const std::vector<Eigen::Vector2d>& points2D,
                            const std::vector<Eigen::Vector3d>& points3D,
                            Eigen::Matrix3x4d* proj_matrix);

        /**
         * @brief calculate relative rotation & translation with the scale of depth
         * from two sets of point clouds
         * @arg 
         * V: list of line of sight projection for each pixel, eqn. 6
         * Tfact: the factor to get optimal translation 
         * up to the current rotation, eqn. 20
        */
        bool CalcLHMRotTrans(const std::vector<Eigen::Vector3d>& points3D0,
                             const std::vector<Eigen::Vector3d>& points3D1,
                             const std::vector<Eigen::Matrix3d>& V,
                             const Eigen::Matrix3d& Tfact,
                             Eigen::Matrix3d& R,
                             Eigen::Vector3d& t);

        /**
         * @brief get the translation via current estimated rotation
        */
        void TransFromRotLHM(const std::vector<Eigen::Vector3d>& points3D,
                             const std::vector<Eigen::Matrix3d>& V,
                             const Eigen::Vector3d& Tfact,
                             const Eigen::Matrix3d& R,
                             Eigen::Vector3d& t);

        /**
         * @brief calculate the collinearity error, eqn. 17-19
         * @arg points3D: here the points are transformed points 
         * via current estimated rotation and translation
        */
        double ObjSpaceLHMErr(const std::vector<Eigen::Vector3d>& points3D,
                              const std::vector<Eigen::Matrix3d>& V);

        /**
         * @brief calculate the standard reprojection error
        */
        double ImgSpaceLHMErr(const std::vector<Eigen::Vector2d>& points2D,
                              const std::vector<Eigen::Vector3d>& points3D,
                              Eigen::Matrix3d& R,
                              Eigen::Vector3d& t);

        /**
         * @brief calculate relative rotation and translation up to a scale of
         * depth for the initial guess of the pose via Weak Perspective model
         * the registration using closed form method w/ quat and profile matrix:
         * Horn, "Closed-form Solution of Absolute Orientation Using Unit Quaternions",
         * JOSAA (4):4, 1987, pp.629
        */
        bool WeakPerspectiveQuat(const std::vector<Eigen::Vector3d>& points3D0,
                                 const std::vector<Eigen::Vector3d>& points3D1,
                                 Eigen::Matrix3d& R,
                                 Eigen::Vector3d& t);

        LHMOptions options_;
};

#endif  // ESTIMATE_LHM_H_
