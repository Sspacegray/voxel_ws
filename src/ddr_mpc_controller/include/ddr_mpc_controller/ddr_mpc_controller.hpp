#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace ddr_navigation
{
class DdrMpcController : public nav2_core::Controller
{
public:
  DdrMpcController();
  ~DdrMpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
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

protected:
  struct ArcPose
  {
    Eigen::Vector2d position;
    double yaw;
    double arc_length;
  };

  struct Projection
  {
    size_t index;
    double squared_distance;
  };

  void rebuildPathCache();
  Projection findClosestPoint(const geometry_msgs::msg::PoseStamped & pose) const;
  ArcPose interpolateAtLength(double target_length) const;
  struct ReferenceSignals
  {
    std::vector<double> linear;
    std::vector<double> angular;
  };

  ReferenceSignals buildReferenceSignals(
    double current_arc,
    double current_yaw,
    const Eigen::Vector2d & current_position) const;

  bool solveVelocityMpc(
    const std::vector<double> & ref_v,
    const std::vector<double> & ref_w,
    double & cmd_v,
    double & cmd_w) const;

#ifdef DDR_MPC_HAS_OSQP
  bool solveWithOsqp(
    const std::vector<double> & ref_v,
    const std::vector<double> & ref_w,
    double & cmd_v,
    double & cmd_w) const;
#endif
  void solveWithCascade(
    const std::vector<double> & ref_v,
    const std::vector<double> & ref_w,
    double & cmd_v,
    double & cmd_w) const;

  double normalizeAngle(double angle) const;
  double clamp(double value, double low, double high) const;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  std::string global_frame_;
  std::string base_frame_;

  nav_msgs::msg::Path global_plan_;
  std::vector<double> path_lengths_;

  double horizon_dt_ = 0.05;
  int horizon_steps_ = 20;
  double nominal_speed_ = 0.8;
  double min_speed_ = -0.3;
  double max_speed_ = 1.2;
  double max_acc_ = 1.5;
  double max_omega_ = 1.5;
  double max_domega_ = 3.0;
  double q_vel_ = 4.0;
  double q_omega_ = 2.0;
  double r_vel_ = 0.5;
  double r_omega_ = 0.2;
  double stopping_distance_ = 0.5;
  double goal_lin_tolerance_ = 0.05;
  double goal_ang_tolerance_ = 0.05;
  double speed_limit_scale_ = 1.0;

  mutable double last_cmd_v_ = 0.0;
  mutable double last_cmd_w_ = 0.0;

  struct CarICR
  {
    double xv = 0.0;
    double yl = 0.0;
    double yr = 0.0;
  };
  CarICR car_icr_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr icr_sub_;
  void icrCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
};
}  // namespace ddr_navigation
