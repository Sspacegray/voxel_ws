#ifndef PP_CONTROLLER__NAV2_PURE_PURSUIT_CONTROLLER_HPP_
#define PP_CONTROLLER__NAV2_PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace pp_controller
{

class Nav2PurePursuitController : public nav2_core::Controller
{
public:
  Nav2PurePursuitController() = default;
  ~Nav2PurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed) const;

  geometry_msgs::msg::PoseStamped getLookAheadPose(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot_pose_in_plan_frame,
    const double lookahead_dist) const;

  static double poseDistance2D(
    const geometry_msgs::msg::PoseStamped & p1,
    const geometry_msgs::msg::PoseStamped & p2);

  bool transformPose(
    const std::string & target_frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("pp_controller.Nav2PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;

  std::string global_frame_;
  std::string base_frame_;

  // Parameters (intentionally aligned with Nav2 RPP where possible)
  double desired_linear_vel_{0.5};
  double base_desired_linear_vel_{0.5};

  double lookahead_dist_{0.6};
  double max_lookahead_dist_{0.9};
  double min_lookahead_dist_{0.3};
  double lookahead_time_{1.5};
  bool use_velocity_scaled_lookahead_dist_{false};

  double min_approach_linear_velocity_{0.05};
  double approach_velocity_scaling_dist_{0.6};

  bool use_regulated_linear_velocity_scaling_{true};
  double regulated_linear_scaling_min_radius_{0.9};
  double regulated_linear_scaling_min_speed_{0.25};

  bool use_rotate_to_heading_{false};
  double rotate_to_heading_min_angle_{0.785};
  double rotate_to_heading_angular_vel_{1.8};

  bool allow_reversing_{false};

  double max_angular_vel_{1.0};
  tf2::Duration transform_tolerance_{tf2::durationFromSec(0.1)};

  nav_msgs::msg::Path global_plan_;
  mutable std::mutex mutex_;
};

}  // namespace pp_controller

#endif  // PP_CONTROLLER__NAV2_PURE_PURSUIT_CONTROLLER_HPP_

