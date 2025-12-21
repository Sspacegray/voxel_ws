#include "pp_controller/nav2_pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.h"
#include "tf2/utils.h"

namespace pp_controller
{

void Nav2PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = std::move(name);
  node_ = parent;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Nav2PurePursuitController: failed to lock parent node");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  global_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(desired_linear_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(lookahead_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(min_lookahead_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(max_lookahead_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(lookahead_time_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(use_velocity_scaled_lookahead_dist_));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity",
    rclcpp::ParameterValue(min_approach_linear_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist",
    rclcpp::ParameterValue(approach_velocity_scaling_dist_));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_regulated_linear_velocity_scaling",
    rclcpp::ParameterValue(use_regulated_linear_velocity_scaling_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_radius",
    rclcpp::ParameterValue(regulated_linear_scaling_min_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_speed",
    rclcpp::ParameterValue(regulated_linear_scaling_min_speed_));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(use_rotate_to_heading_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_min_angle",
    rclcpp::ParameterValue(rotate_to_heading_min_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel",
    rclcpp::ParameterValue(rotate_to_heading_angular_vel_));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(allow_reversing_));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(max_angular_vel_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;

  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);

  node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);

  node->get_parameter(plugin_name_ + ".use_regulated_linear_velocity_scaling",
    use_regulated_linear_velocity_scaling_);
  node->get_parameter(plugin_name_ + ".regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_);
  node->get_parameter(plugin_name_ + ".regulated_linear_scaling_min_speed", regulated_linear_scaling_min_speed_);

  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);

  node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);

  double transform_tolerance_sec = 0.1;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_sec);
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance_sec);

  RCLCPP_INFO(
    logger_,
    "[%s] configured (frames: global=%s base=%s, lookahead=[%.2f..%.2f], desired_v=%.2f)",
    plugin_name_.c_str(), global_frame_.c_str(), base_frame_.c_str(),
    min_lookahead_dist_, max_lookahead_dist_, desired_linear_vel_);
}

void Nav2PurePursuitController::cleanup()
{
  std::scoped_lock<std::mutex> lock(mutex_);
  global_plan_ = nav_msgs::msg::Path();
}

void Nav2PurePursuitController::activate()
{
}

void Nav2PurePursuitController::deactivate()
{
}

void Nav2PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  global_plan_ = path;
}

double Nav2PurePursuitController::poseDistance2D(
  const geometry_msgs::msg::PoseStamped & p1,
  const geometry_msgs::msg::PoseStamped & p2)
{
  return nav2_util::geometry_utils::euclidean_distance(p1, p2, false);
}

bool Nav2PurePursuitController::transformPose(
  const std::string & target_frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (!tf_) {
    return false;
  }
  // Use time 0 to get the latest available transform, avoiding sim_time mismatch
  geometry_msgs::msg::PoseStamped pose_at_time_zero = in_pose;
  pose_at_time_zero.header.stamp = rclcpp::Time(0);
  return nav2_util::transformPoseInTargetFrame(
    pose_at_time_zero, out_pose, *tf_, target_frame, tf2::durationToSec(transform_tolerance_));
}

double Nav2PurePursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed) const
{
  double ld = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    ld = std::abs(speed.linear.x) * lookahead_time_;
  }
  ld = std::clamp(ld, min_lookahead_dist_, max_lookahead_dist_);
  return ld;
}

geometry_msgs::msg::PoseStamped Nav2PurePursuitController::getLookAheadPose(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose_in_plan_frame,
  const double lookahead_dist) const
{
  if (path.poses.empty()) {
    throw std::runtime_error("Nav2PurePursuitController: empty plan");
  }

  size_t closest_idx = 0;
  double closest_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double d = poseDistance2D(robot_pose_in_plan_frame, path.poses[i]);
    if (d < closest_dist) {
      closest_dist = d;
      closest_idx = i;
    }
  }

  if (closest_idx >= path.poses.size() - 1) {
    return path.poses.back();
  }

  double accumulated = 0.0;
  for (size_t i = closest_idx; i < path.poses.size() - 1; ++i) {
    const auto & p1 = path.poses[i];
    const auto & p2 = path.poses[i + 1];
    const double seg_len = poseDistance2D(p1, p2);
    if (seg_len < 1e-9) {
      continue;
    }

    if (accumulated + seg_len >= lookahead_dist) {
      const double remain = lookahead_dist - accumulated;
      const double ratio = std::clamp(remain / seg_len, 0.0, 1.0);

      geometry_msgs::msg::PoseStamped carrot = p2;
      carrot.header = path.header;
      carrot.pose.position.x = p1.pose.position.x + ratio * (p2.pose.position.x - p1.pose.position.x);
      carrot.pose.position.y = p1.pose.position.y + ratio * (p2.pose.position.y - p1.pose.position.y);
      carrot.pose.position.z = 0.0;
      return carrot;
    }

    accumulated += seg_len;
  }

  return path.poses.back();
}

geometry_msgs::msg::TwistStamped Nav2PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  nav_msgs::msg::Path plan;
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    plan = global_plan_;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_ ? clock_->now() : rclcpp::Clock().now();
  cmd.header.frame_id = base_frame_;

  if (plan.poses.empty()) {
    return cmd;
  }

  geometry_msgs::msg::PoseStamped pose_in_plan = pose;
  if (!plan.header.frame_id.empty() && pose_in_plan.header.frame_id != plan.header.frame_id) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    if (!transformPose(plan.header.frame_id, pose_in_plan, transformed_pose)) {
      throw std::runtime_error(
              "Nav2PurePursuitController: failed to transform robot pose to plan frame");
    }
    pose_in_plan = transformed_pose;
  }

  const double lookahead = getLookAheadDistance(velocity);
  const auto carrot_in_plan = getLookAheadPose(plan, pose_in_plan, lookahead);

  geometry_msgs::msg::PoseStamped carrot_in_base;
  if (!transformPose(base_frame_, carrot_in_plan, carrot_in_base)) {
    throw std::runtime_error("Nav2PurePursuitController: failed to transform carrot to base frame");
  }

  const double carrot_x = carrot_in_base.pose.position.x;
  const double carrot_y = carrot_in_base.pose.position.y;
  const double carrot_dist = std::hypot(carrot_x, carrot_y);
  if (carrot_dist < 1e-6) {
    return cmd;
  }

  const double angle_to_carrot = std::atan2(carrot_y, carrot_x);
  if (use_rotate_to_heading_ && std::abs(angle_to_carrot) > rotate_to_heading_min_angle_) {
    cmd.twist.angular.z = std::clamp(
      std::copysign(rotate_to_heading_angular_vel_, angle_to_carrot),
      -max_angular_vel_, max_angular_vel_);
    return cmd;
  }

  // Base linear velocity
  double linear_mag = std::max(0.0, desired_linear_vel_);

  // Approach scaling to goal
  if (approach_velocity_scaling_dist_ > 1e-6) {
    const auto & goal = plan.poses.back();
    const double dist_to_goal = poseDistance2D(pose_in_plan, goal);
    if (dist_to_goal < approach_velocity_scaling_dist_) {
      const double scale = std::clamp(dist_to_goal / approach_velocity_scaling_dist_, 0.0, 1.0);
      linear_mag = std::max(linear_mag * scale, min_approach_linear_velocity_);
    }
  }

  // Curvature-based regulation (no collision checking)
  double curvature = 2.0 * carrot_y / (carrot_dist * carrot_dist);
  if (use_regulated_linear_velocity_scaling_) {
    const double radius = 1.0 / std::max(std::abs(curvature), 1e-6);
    if (radius < regulated_linear_scaling_min_radius_) {
      const double scale = std::clamp(radius / regulated_linear_scaling_min_radius_, 0.0, 1.0);
      linear_mag = std::max(linear_mag * scale, regulated_linear_scaling_min_speed_);
    }
  }

  // Reversing support (simple heuristic)
  double sign = 1.0;
  if (allow_reversing_ && carrot_x < 0.0) {
    sign = -1.0;
    curvature = -curvature;
  }

  cmd.twist.linear.x = sign * linear_mag;
  cmd.twist.angular.z = std::clamp(cmd.twist.linear.x * curvature, -max_angular_vel_, max_angular_vel_);

  return cmd;
}

void Nav2PurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    desired_linear_vel_ = base_desired_linear_vel_ * (speed_limit / 100.0);
  } else {
    desired_linear_vel_ = speed_limit;
  }

  desired_linear_vel_ = std::max(0.0, desired_linear_vel_);
}

}  // namespace pp_controller

PLUGINLIB_EXPORT_CLASS(pp_controller::Nav2PurePursuitController, nav2_core::Controller)
