#ifndef ESTIMATE_OPNP_H_
#define ESTIMATE_OPNP_H_

#include <Eigen/Core>
#include <vector>

class OPnPEstimator {
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
     * @brief implement of OPnP
     */
    bool ComputeOPnPPose(const std::vector<Eigen::Vector2d>& points2D,
                         const std::vector<Eigen::Vector3d>& points3D,
                         Eigen::Matrix3x4d* proj_matrix);

    struct TemplateData {
        uint8_t dim;           // dimension of p-fold solutions (40)
        uint8_t basis_size;    // size of basis (120)
        std::vector<uint16_t> P;  // permissible monomials
        std::vector<uint16_t> ind;  // indices
        std::vector<uint16_t> R;  // reducible monomials
        std::vector<uint16_t> E;  // excessive monomials
        std::vector<uint16_t> ids_a1;
        std::vector<uint16_t> ids_a3;
        std::vector<uint16_t> ids_abc;
        std::vector<uint16_t> ids_ac2;
        std::vector<uint16_t> ids_a2c;
        std::vector<uint16_t> I;
        std::vector<uint16_t> J;
        std::vector<uint16_t> II;
        std::vector<uint16_t> JJ;
        std::vector<uint8_t> reorder;
        std::vector<std::vector<uint8_t>> mon;  // monomial exponents
        std::string p_inverter;  // "all"
        std::vector<uint8_t> T;
        std::vector<uint32_t> Tid;
    };
    
    /**Create solver settings (mirroring MATLAB settings)
     */
    struct SolverSettings {
        bool debug = false;
        bool full_QR1 = false;
        std::string p_inverter = "all";
        bool real_solutions_only = true;
        double cutoff_threshold = 1e12;
        double inverter_threshold = 1.0;
        // Template data
        OPnPEstimator::TemplateData template_data;  // Include template data in settings

    };

    // Static template data shared across all instances
    static TemplateData template_data_;

    // Static function to get template data (loads only once)
    static void getTemplateData();


private:
    bool solveGrobnerBasis(
        const Eigen::VectorXd& coeff1,
        const Eigen::VectorXd& coeff2,
        const Eigen::VectorXd& coeff3,
        const Eigen::VectorXd& coeff4,
        std::vector<double>& x,
        std::vector<double>& y,
        std::vector<double>& z,
        std::vector<double>& t);
    
    void filterRepetitiveSolutions(
            std::vector<double>& x,
            std::vector<double>& y,
            std::vector<double>& z,
            std::vector<double>& t);
    
    // The actual solver (mirrors solver_pfold)
    bool solverPFold(
        const Eigen::MatrixXd& coefficients,
        const SolverSettings& settings,
        Eigen::MatrixXd& solutions,
        Eigen::MatrixXd& stats);

    bool polishSolution(const Eigen::MatrixXd& Q, 
                   const Eigen::VectorXd& q,
                   Eigen::Vector4d& solution,
                   bool& label_psd,
                   double& obj_cur);

    void loadTemplateData(TemplateData& tmpl);

};

#endif  // ESTIMATE_OPNP_H_