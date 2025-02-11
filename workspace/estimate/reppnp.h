#ifndef ESTIMATE_REPPNP_H_
#define ESTIMATE_REPPNP_H_

#include <Eigen/Core>
#include <vector>

class REPPnPEstimator {
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
     * @brief implementation of REPPnP
     * @param min_error Optional minimum error threshold for robust estimation (default: 0.02)
     */
    bool ComputeREPPnPPose(const std::vector<Eigen::Vector2d>& points2D,
                           const std::vector<Eigen::Vector3d>& points3D,
                           Eigen::Matrix3x4d* proj_matrix,
                           double min_error = 0.02);
    private:
        void PrepareData(Eigen::MatrixXd& alphas,          
                        Eigen::Matrix<double, 4, 3>& control_points,
                        Eigen::MatrixXd& M,
                        const std::vector<Y_t>& points3D,
                        const std::vector<X_t>& points2D);

        void ComputeM(Eigen::MatrixXd& M,
                      const Eigen::VectorXd& U,
                      const Eigen::MatrixXd& alphas);

        void RobustKernelEstimation(
                    Eigen::MatrixXd& Km,                // Output: Kernel matrix
                    std::vector<int>& inliers,          // Output: Indices of inlier points (changed from size_t to int)
                    int& robust_iters,                  // Output: Number of iterations
                    const Eigen::MatrixXd& M,           // Input: M matrix
                    int dims,                           // Input: Kernel dimensions
                    double min_error);                  // Input: Minimum error threshold

        /**
         * @brief Compute the pose from the kernel
         * @param control_points: transposed matrix from DefineControlPoints
         */
        bool ComputePoseFromKernel(Eigen::Matrix3d& R,                    
                                    Eigen::Vector3d& t,                   
                                    double& error,                         
                                    const Eigen::Matrix<double, 3, 4>& control_points, 
                                    const Eigen::MatrixXd& Km,           
                                    int dims,                             
                                    bool sol_iter);  

        /**
         * @brief Procrustes to align points
         */
        void MyProcrustes(const Eigen::Matrix<double, 3, Eigen::Dynamic>& P,     // Original points
                                            const Eigen::Vector3d& mP,                             // Mean of points
                                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& nP,    // Normalized points
                                            const double norm,                                     // Norm of centered points
                                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& Y,     // Points to align with
                                            Eigen::Matrix3d& R,                                    // Output rotation
                                            double& b,                                             // Output scale
                                            Eigen::Matrix<double, 3, Eigen::Dynamic>& mc);         // Output mean-centered                  
                                        
        // Define control points
        void DefineControlPoints(Eigen::Matrix<double, 4, 3>& control_points); 

        // Define alphas
        void ComputeAlphas(Eigen::MatrixXd& alphas,
                           const std::vector<Y_t>& points3D,
                           const Eigen::Matrix<double, 4, 3>& control_points);


};

#endif  // ESTIMATE_REPPNP_H_