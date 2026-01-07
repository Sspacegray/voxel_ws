#pragma once

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "industrial_route_core/types.hpp"

namespace industrial_route_ros2 {

class PurePursuitController {
public:
  struct Limits {
    double max_linear_speed{0.5};
    double max_angular_speed{1.2};
    double max_linear_accel{0.6};
    double max_angular_accel{2.0};
  };

  struct Params {
    double lookahead_min{0.4};
    double lookahead_gain{0.8};
    double goal_pos_tolerance{0.15};
    double goal_yaw_tolerance{0.25};
    double rotate_in_place_yaw_threshold{0.6};
    double rotate_in_place_speed{0.5};
  };

  struct State {
    std::size_t closest_index{0};
    double progress{0.0};
    bool reached_goal{false};
  };

  PurePursuitController(const Limits& limits, const Params& params)
      : limits_(limits), params_(params) {}

  void setPath(const std::vector<industrial_route_core::Pose2D>& path);

  geometry_msgs::msg::Twist computeCommand(const industrial_route_core::Pose2D& current_pose,
                                           double current_linear_speed,
                                           bool rotate_in_place,
                                           State* state);

private:
  std::size_t findClosestIndex(const industrial_route_core::Pose2D& current_pose,
                               std::size_t from_index) const;
  std::size_t findLookaheadIndex(const industrial_route_core::Pose2D& current_pose,
                                 std::size_t closest_index,
                                 double lookahead_dist) const;
  geometry_msgs::msg::Twist clamp(const geometry_msgs::msg::Twist& cmd,
                                 const geometry_msgs::msg::Twist& prev,
                                 double dt) const;

  Limits limits_;
  Params params_;
  std::vector<industrial_route_core::Pose2D> path_;
  geometry_msgs::msg::Twist last_cmd_;
};

}  // namespace industrial_route_ros2

