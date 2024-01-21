#include <string>
#include <Eigen/Core>
#include <map>

/**
 * @brief convert processed kitti odometry's extrinsic g.t. to a 
 * map of quaternion & translation,
 * the required g.t. frame's indices are specified in gt_nums
 * @arg ground_truth: the g.t. map to be filled
*/
void CreateGTMap(std::map<int, std::pair<Eigen::Vector4d,Eigen::Vector3d>>& ground_truth,
                 std::vector<int>& gt_nums, 
                 std::vector<std::vector<double>>& processed_extrinsic);