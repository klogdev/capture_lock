#include <iostream>
#include <vector>

struct state {
    double x = 0;    // Position
    double vx = 0;   // Speed along x
    double ax = 0;   // Acceleration along x
};

class LaneChangeFeature {
public:
    void checkLaneChangeAllowed(state hero, std::vector<state> entities) {
    // Define safe distances
    static const double Df = 20.0;  // Safe distance to car in front
    static const double Db = 15.0;  // Safe distance to car behind
    
    for(const state& entity : entities) {
        // Check forward
        if(entity.x > hero.x && (entity.x - hero.x) < Df) {
            lane_change_active_ = false;
            return;
        }

        // Check backward
        if(entity.x < hero.x && (hero.x - entity.x) < Db) {
            // Additional check: if the entity's speed is much greater than hero's, don't change lanes
            if(entity.vx > hero.vx + 5.0) {  // 5 m/s is a threshold, can be adjusted
                lane_change_active_ = false;
                return;
            }
        }
    }
    
    // If neither condition met, safe to change lanes
    lane_change_active_ = true;
}


    void selectRelevantTargets(state hero, std::vector<state> entities) {
    // Buffer distances to consider a car relevant even if it's going faster
    static const double forward_buffer_distance = 20.0; // meters
    static const double rearward_buffer_distance = 15.0; // meters
    
    // Start with an empty list of relevant targets
    relevant_following_targets_.clear();

    state* forward_car = nullptr;
    state* rearward_car = nullptr;

    // Identify relevant cars
    for (state& entity : entities) {
        // Car is ahead
        if (entity.x > hero.x) {
            // If we haven't identified a forward car yet or this car is closer than the current forward car
            if (!forward_car || (entity.x - hero.x) < (forward_car->x - hero.x)) {
                // The car is moving slower than the hero or is within the buffer distance
                if (entity.vx <= hero.vx || (entity.x - hero.x) <= forward_buffer_distance) {
                    forward_car = &entity;
                }
            }
        }
        // Car is behind
        else if (entity.x < hero.x) {
            // If we haven't identified a rearward car yet or this car is closer than the current rearward car
            if (!rearward_car || (hero.x - entity.x) < (hero.x - rearward_car->x)) {
                                // The car is moving faster than the hero and is within the buffer distance
                if (entity.vx > hero.vx && (hero.x - entity.x) <= rearward_buffer_distance) {
                    rearward_car = &entity;
                }
            }
        }
    }

    // Add relevant cars to our list
    if (forward_car) {
        relevant_following_targets_.push_back(*forward_car);
    }
    if (rearward_car) {
        relevant_following_targets_.push_back(*rearward_car);
    }
}


double calculateAx(state hero, double vx_speed_limit) {
    // Desired acceleration based on the road speed limit
    double ax_desired = velocityController(hero.vx, vx_speed_limit);
    
    // For each relevant target, calculate the desired following acceleration
    for(const state& target : relevant_following_targets_) {
        double relative_speed = target.vx - hero.vx;
        double relative_distance = target.x - hero.x;
        
        // Use followController to get desired acceleration for this target
        double ax_follow = followController(relative_distance, safe_following_distance, relative_speed);

        // Adjust the desired acceleration if it's more conservative than the current one
        if (ax_follow < ax_desired) {
            ax_desired = ax_follow;
        }
    }

    return ax_desired;
}


private:
    double velocityController(double vx_current, double vx_desired) const {
        static constexpr double kp = 0.5;
        vx_current = std::max(vx_current, 0.0);
        return kp * (vx_desired - vx_current);
    }

    double followController(double distance, double distance_desired, double vx_relative) const {
        static constexpr double kp_vx = 0.5;
        static constexpr double kp_ax = 0.5;
        return -kp_ax * (kp_vx * (distance_desired - distance) - vx_relative);
    }

    bool lane_change_active_ = false;
    std::vector<state> relevant_following_targets_;
    static constexpr double safe_following_distance = 10.0;  // in meters, for example

};

int main() {
    state hero = {30, 10, 0};
    std::vector<state> entities = {{20, 12, 0}, {50, 14, 0}};
    static constexpr double vx_speed_limit = 20;

    double t = 0;
    double t_end = 10;
    double dt = 0.1;

    LaneChangeFeature lc_feature;
    while (t < t_end) {
        // Implement the simulation logic here

        std::cout << "t " << t << " x (" << hero.x;
        for (const auto& entity : entities) {
            std::cout << ", " << entity.x;
        }
        std::cout << ") vx (" << hero.vx;
        for (const auto& entity : entities) {
            std::cout << ", " << entity.vx;
        }
        // Continue printing as needed

        t += dt;
    }

    return 0;
}
