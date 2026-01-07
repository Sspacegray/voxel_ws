#include "industrial_route_ros2/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>

namespace industrial_route_ros2 {
namespace {

double clampAbs(double v, double max_abs) {
  if (v > max_abs) return max_abs;
  if (v < -max_abs) return -max_abs;
  return v;
}

double hypot2(double x, double y) { return std::sqrt(x * x + y * y); }

}  // namespace

void PurePursuitController::setPath(const std::vector<industrial_route_core::Pose2D>& path) {
  path_ = path;
  last_cmd_ = geometry_msgs::msg::Twist{};
}

std::size_t PurePursuitController::findClosestIndex(const industrial_route_core::Pose2D& current_pose,
                                                    std::size_t from_index) const {
  if (path_.empty()) return 0;
  from_index = std::min(from_index, path_.size() - 1);

  std::size_t best = from_index;
  double best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = from_index; i < path_.size(); ++i) {
    const double dx = path_[i].x - current_pose.x;
    const double dy = path_[i].y - current_pose.y;
    const double d = dx * dx + dy * dy;
    if (d < best_d) {
      best_d = d;
      best = i;
    }
    // early exit if we start getting further (path is ordered)
    if (i > best + 20) break;
  }
  return best;
}

std::size_t PurePursuitController::findLookaheadIndex(const industrial_route_core::Pose2D& current_pose,
                                                      std::size_t closest_index,
                                                      double lookahead_dist) const {
  if (path_.empty()) return 0;
  closest_index = std::min(closest_index, path_.size() - 1);
  const double L2 = lookahead_dist * lookahead_dist;
  for (std::size_t i = closest_index; i < path_.size(); ++i) {
    const double dx = path_[i].x - current_pose.x;
    const double dy = path_[i].y - current_pose.y;
    if (dx * dx + dy * dy >= L2) return i;
  }
  return path_.size() - 1;
}

geometry_msgs::msg::Twist PurePursuitController::clamp(const geometry_msgs::msg::Twist& cmd,
                                                       const geometry_msgs::msg::Twist& prev,
                                                       double dt) const {
  geometry_msgs::msg::Twist out = cmd;
  out.linear.x = clampAbs(out.linear.x, limits_.max_linear_speed);
  out.angular.z = clampAbs(out.angular.z, limits_.max_angular_speed);

  if (dt <= 1e-3) return out;

  const double dv = out.linear.x - prev.linear.x;
  const double dw = out.angular.z - prev.angular.z;
  const double max_dv = limits_.max_linear_accel * dt;
  const double max_dw = limits_.max_angular_accel * dt;

  out.linear.x = prev.linear.x + clampAbs(dv, max_dv);
  out.angular.z = prev.angular.z + clampAbs(dw, max_dw);
  return out;
}

geometry_msgs::msg::Twist PurePursuitController::computeCommand(const industrial_route_core::Pose2D& current_pose,
                                                                double current_linear_speed,
                                                                bool rotate_in_place,
                                                                State* state) {
  geometry_msgs::msg::Twist cmd;
  if (!state) return cmd;
  if (path_.empty()) {
    state->reached_goal = true;
    state->progress = 1.0;
    return cmd;
  }

  const auto& goal = path_.back();
  const double dxg = goal.x - current_pose.x;
  const double dyg = goal.y - current_pose.y;
  const double dist_to_goal = hypot2(dxg, dyg);

  state->closest_index = findClosestIndex(current_pose, state->closest_index);
  state->progress = static_cast<float>(state->closest_index) / static_cast<float>(std::max<std::size_t>(1, path_.size() - 1));

  const double yaw_err_goal = industrial_route_core::shortestAngularDistance(current_pose.yaw, goal.yaw);
  if (dist_to_goal <= params_.goal_pos_tolerance &&
      std::abs(yaw_err_goal) <= params_.goal_yaw_tolerance) {
    state->reached_goal = true;
    state->progress = 1.0;
    last_cmd_ = geometry_msgs::msg::Twist{};
    return geometry_msgs::msg::Twist{};
  }

  const double lookahead = std::max(params_.lookahead_min, params_.lookahead_gain * std::max(0.0, current_linear_speed));
  const auto look_idx = findLookaheadIndex(current_pose, state->closest_index, lookahead);
  const auto& target = path_[look_idx];

  const double dx = target.x - current_pose.x;
  const double dy = target.y - current_pose.y;
  const double target_heading = std::atan2(dy, dx);
  const double alpha = industrial_route_core::shortestAngularDistance(current_pose.yaw, target_heading);

  // Rotate-in-place mode: useful for diff-drive in tight industrial corridors.
  if (rotate_in_place && std::abs(alpha) >= params_.rotate_in_place_yaw_threshold) {
    cmd.linear.x = 0.0;
    cmd.angular.z = clampAbs((alpha > 0.0 ? 1.0 : -1.0) * params_.rotate_in_place_speed, limits_.max_angular_speed);
    cmd = clamp(cmd, last_cmd_, 0.05);
    last_cmd_ = cmd;
    return cmd;
  }

  // Pure pursuit curvature and speed
  const double L = std::max(lookahead, 1e-3);
  const double curvature = (2.0 * std::sin(alpha)) / L;

  double v = limits_.max_linear_speed;
  // Slow down near goal
  v *= std::clamp(dist_to_goal / 1.0, 0.2, 1.0);
  // Slow down on sharp turns
  v *= std::clamp(1.0 - std::abs(curvature) * 0.5, 0.2, 1.0);

  cmd.linear.x = v;
  cmd.angular.z = clampAbs(curvature * v, limits_.max_angular_speed);

  cmd = clamp(cmd, last_cmd_, 0.05);
  last_cmd_ = cmd;
  return cmd;
}

}  // namespace industrial_route_ros2
