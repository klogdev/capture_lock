#ifndef ESTIMATE_LHM_H_
#define ESTIMATE_LHM_H_

#include <vector>

#include <Eigen/Core>

#include "base/camera.h"
#include "estimate/adj_quat.h"

struct LHMOptions
{  
    // convergence tolerance of lhm
    double lhm_tolerance = 1e-5;

    // lower bound of objective space error for the termination
    double lhm_epsilon = 1e-9;

    // max iteration for lhm's optimization
    int lhm_iter = 15;

    // option to use DRaM & Bar-Itzhack as initial guess for rotation
    // or standard SVD et.al.
    std::string rot_init_est = "horn";

    // optimization method for the iteration, can be "lhm" or "gn"
    std::string optim_option = "lhm";
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
        static const int kMinNumSamples = 4;

        // Estimate the most probable solution of the P3P problem from a set of
        // three 2D-3D point correspondences.
        //
        // @param points2D   Normalized 2D image points as 3x2 matrix.
        // @param points3D   3D world points as 3x3 matrix.
        //
        // @return           Most probable pose as length-1 vector of a 3x4 matrix.
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
         * @brief set the g.t. pose to compare with the first estimation
         * by DRaM; we use static memeber due to RANSAC template does
         * not allowed a direct constructor arg
        */
        static void setGroundTruthPose(Eigen::Matrix3x4d* gt_pose);

        /**
         * @brief set the options for LHM
         * @note need to pay attention to thread safety if run
         * with multithreading
        */
        static void setGlobalOptions(const LHMOptions& options);

        /**
         * @brief set the first estimated frobenius norm between DRaM or Horn
         *  and the g.t.
        */
        static void setFirstFrob(const double frob);

        /**
         * @brief set the first estimated relative quaternion error
         * between DRaM or Horn estimated and the g.t.
        */
        static void setFirstRelaQuat(const double quat_err);

        /**
         * @brief set the number of iterations to be read as a metric
        */
        static void setNumIters(const int iters);

        /**
         * @brief the g.t. pose as a 3x4 extrinsic matrix
        */
        static Eigen::Matrix3x4d* gt_pose_;

        /**
         * @brief init options for LHM manually; 
         * should implement a helper function to save the option that currently using
        */
        static LHMOptions options_;

        /**
         * @brief Frobenius norm w.r.t the g.t. after initial guess
         * will be overwrote after DRaM estimation
        */
        static double first_estimated_frob;

        /**
         * @brief relative quaternion error w.r.t the g.t. 
         * after initial guess
        */
        static double first_estimated_rela_quat;

        /**
         * @brief number of iterations of GN or LHM
         * will be overwrote after iterations finished
        */
        static int num_iterations;

        /**
         * @brief object space error during the LHM iterations
        */
        static std::vector<double> obj_errs;

        /**
         * @brief relative quaternion difference during iteration
         */
        static std::vector<double> rel_quats;

        /**
         * @brief append current relative quaternion error to
         * the static container
         */
        static void addRelQuats(const double rel_quat);

        /**
         * @brief append current object space error to 
         * the static container
         */
        static void addObjErrs(const double obj_err);

        /**
         * @brief estimate the absolute pose via LHM from corresponded 
         * 2D, 3D points, here we adopt COLMAP's convention that the 
         * 2D points are normalized points in the camera space
        */
        bool ComputeLHMPose(const std::vector<Eigen::Vector2d>& points2D,
                            const std::vector<Eigen::Vector3d>& points3D,
                            Eigen::Matrix3x4d* proj_matrix);

    private:
        /**
         * @brief calculate relative rotation & translation with the scale of depth
         * from two sets of point clouds
         * @arg 
         * V: list of line of sight projection for each pixel, eqn.6
         * Tfact: the factor to get optimal translation 
         * up to the current rotation, eqn.20
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
                             const Eigen::Matrix3d& Tfact,
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
         * @brief iterative optimization by using LHM's pipeline
         * @return number of iterations
        */
        int IterationLHM(const std::vector<Eigen::Vector3d>& points3D,
                          const std::vector<Eigen::Matrix3d>& V,
                          const Eigen::Matrix3d& Tfact,
                          Eigen::Matrix3d& init_rot,
                          Eigen::Vector3d& init_trans);

        /**
         * @brief calculate relative rotation and translation up to a scale of
         * depth for the initial guess of the pose via Weak Perspective model
         * the registration using closed form method w/ quat and profile matrix:
         * Horn, "Closed-form Solution of Absolute Orientation Using Unit Quaternions",
         * JOSAA (4):4, 1987, pp.629
         * we always use eqn. 39 for the scale w/o any guess
        */
        bool WeakPerspectiveQuat(const std::vector<Eigen::Vector3d>& points3D0,
                                 const std::vector<Eigen::Vector3d>& points3D1,
                                 Eigen::Matrix3d& R,
                                 Eigen::Vector3d& t);
        
        /**
         * @brief use adjugate quaternion based solution on
         * 3D-3D registration to have a better init rotation
        */
        bool WeakPerspectiveDRaMInit3D(const std::vector<Eigen::Vector3d>& points3D0,
                                       const std::vector<Eigen::Vector3d>& points3D1,
                                       Eigen::Matrix3d& rot_opt,
                                       Eigen::Vector3d& trans_init);

        /**
         * @brief use adjugate quaternion based solution on
         * 3D-2D registration to have a better init rotation
         * with the Bar-Itzhack correction.
         * to be consistent with other initialization method, we pass 
         * the 3D homogeneaous coordinates as input, but will
         * project them back as 2D pixels inside the function
        */
        bool WeakPerspectiveDRaMInit2D(const std::vector<Eigen::Vector3d>& points3D0,
                                       const std::vector<Eigen::Vector3d>& points3D1,
                                       Eigen::Matrix3d& rot_opt,
                                       Eigen::Vector3d& trans_init);

        /**
         * @brief preprocess of the weak perspective model by getting
         * the centroid of both point clouds.
        */
        void GetCentroid(const std::vector<Eigen::Vector3d>& points3D0,
                         const std::vector<Eigen::Vector3d>& points3D1,
                         Eigen::Vector3d& pc, Eigen::Vector3d& qc);
};

#endif  // ESTIMATE_LHM_H_
