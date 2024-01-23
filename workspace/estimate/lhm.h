#ifndef ESTIMATE_LHM_H_
#define ESTIMATE_LHM_H_

#include <vector>

#include <Eigen/Core>

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
        void lhmEstPose(const std::vector<Eigen::Vector2d>& points2D,
                    const std::vector<Eigen::Vector3d>& points3D,
                    std::vector<Y_t>& wkpts3D,
                    const std::vector<int>& ptsidx,
                    Eigen::Matrix3x4d* proj_matrix,
                    bool verbose);

        void lhmCalcRt(const std::vector<Eigen::Vector3d>& points3D0,
                    const std::vector<Eigen::Vector3d>& points3D1,
                    const std::vector<int>& index,
                    const std::vector<Eigen::Matrix3d>& V,
                    const Eigen::Matrix3d& Tfact,
                    Eigen::Matrix3d& R,
                    Eigen::Vector3d& t);

        void lhmTFromR(const std::vector<Eigen::Vector3d>& pts3D,
                    const std::vector<int>& ptsidx,
                    const std::vector<Eigen::Matrix3d>& V,
                    const Eigen::Matrix3d& R,
                    const Eigen::Vector3d& Tfact,
                    Eigen::Vector3d& t);

        double lhmObjSpaceErr(const std::vector<Eigen::Vector3d>& pts3D,
                            const std::vector<int>& ptsidx,
                            const std::vector<Eigen::Matrix3d>& V);

        double lhmImgSpaceErr(const std::vector<Eigen::Vector2d>& pts2D,
                            const std::vector<Eigen::Vector3d>& pts3D,
                            const std::vector<int>& ptsidx,
                            Eigen::Matrix3d& R,
                            Eigen::Vector3d& t);

};

#endif  // ESTIMATE_LHM_H_
