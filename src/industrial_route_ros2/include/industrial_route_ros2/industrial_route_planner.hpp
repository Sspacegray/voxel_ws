// SPDX-License-Identifier: Apache-2.0
// industrial_route_planner.hpp - Nav2 Global Planner plugin for industrial route network

#ifndef INDUSTRIAL_ROUTE_ROS2__INDUSTRIAL_ROUTE_PLANNER_HPP_
#define INDUSTRIAL_ROUTE_ROS2__INDUSTRIAL_ROUTE_PLANNER_HPP_

#include <string>
#include <memory>

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "industrial_route_interfaces/srv/plan_route.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace industrial_route_ros2
{

class IndustrialRoutePlanner : public nav2_core::GlobalPlanner
{
public:
  IndustrialRoutePlanner() = default;
  ~IndustrialRoutePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Client<industrial_route_interfaces::srv::PlanRoute>::SharedPtr plan_client_;
  std::string name_;
  std::string global_frame_;
  double service_timeout_sec_;
  std::string service_name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("industrial_route_planner")};
};

}  // namespace industrial_route_ros2

#endif  // INDUSTRIAL_ROUTE_ROS2__INDUSTRIAL_ROUTE_PLANNER_HPP_
