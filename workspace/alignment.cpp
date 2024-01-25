#include "feature/sift.h"
#include "feature/image_sift.h"
#include "optim/ransac.h"

#include <Eigen/Core>

#include "estimate/epipole_estimator.h"
#include "incremental_construct.h"

Eigen::Vector3d FinalAlignment(std::vector<sift::Keypoint>& locked_keypts, 
                               std::string compensate_path){
    Image compensate_frame(compensate_path);
    std::vector<sift::Keypoint> compensate_keypts = GetKeyPoints(compensate_path);
    //covert sift keypts to eigen
    std::vector<Eigen::Vector2d> lock_keypts_vec = SIFTPtsToVec(locked_keypts);
    std::vector<Eigen::Vector2d> comp_keypts_vec = SIFTPtsToVec(compensate_keypts);

    std::vector<std::pair<int, int>> matches = sift::find_keypoint_matches(locked_keypts, compensate_keypts);

    std::vector<PixelPair> all_pairs1;
    std::vector<PixelPair> all_pairs2;
    for(int i = 0; i < matches.size(); i++){
        Eigen::Vector2d vec_lock = lock_keypts_vec[matches[i].first];
        Eigen::Vector2d vec_comp = comp_keypts_vec[matches[i].second];
        PixelPair curr_pair(vec_lock, comp_lock);
        all_pairs1.push_back(curr_pair);
        all_pairs2.push_back(curr_pair);
    }

    colmap::RANSACOptions ransac_options;
    std::vector<char> inliers;
    Eigen::Vector3d best_epipole;

    bool epipole_ransac = EstimateEpipole(ransac_options, all_pairs1, all_pairs2,
                                          &inliers, &best_epipole);
    std::cout << "result of epipole ransac is: " << epipole_ransac << std::endl;

    return best_epipole;
}