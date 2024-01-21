#include <string>
#include <Eigen/Core>
#include <map>

#include "util_/kitti_pose.h"

void CreateGTMap(std::map<int, std::pair<Eigen::Vector4d,Eigen::Vector3d>>& ground_truth,
                 std::vector<int>& gt_nums, 
                 std::vector<std::vector<double>>& processed_extrinsic){
    
    for(int n: gt_nums){
        const Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> curr_frame(processed_extrinsic[n].data());

        Eigen::Matrix3d curr_R = curr_frame.block<3, 3>(0, 0);
        Eigen::Vector3d curr_tr = curr_frame.col(3);
        Eigen::Vector3d tvec_inv = -curr_R * curr_tr;

        // init the g.t. poses for the current frame
        Eigen::Vector4d qvec_kitti_curr;
        Eigen::Vector3d tvec_kitti_curr;
        QuatTransFromExtrinsic(qvec_kitti_curr, tvec_kitti_curr, curr_frame);

        ground_truth[n] = {qvec_kitti_curr, tvec_inv};
    }
}