// SPDX-License-Identifier: Apache-2.0
// industrial_route_planner.cpp - Nav2 Global Planner plugin for industrial route network

#include "industrial_route_ros2/industrial_route_planner.hpp"
#include <chrono>
#include <memory>
#include "nav2_util/node_utils.hpp"

namespace industrial_route_ros2
{

void IndustrialRoutePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  auto node = node_.lock();
  
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".service_timeout", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".service_name", rclcpp::ParameterValue(std::string("/industrial_route/plan")));
  node->get_parameter(name_ + ".service_timeout", service_timeout_sec_);
  node->get_parameter(name_ + ".service_name", service_name_);

  // Create service client
  plan_client_ = node->create_client<industrial_route_interfaces::srv::PlanRoute>(
    service_name_);

  RCLCPP_INFO(
    logger_,
    "IndustrialRoutePlanner configured, service=%s timeout=%.1fs",
    service_name_.c_str(), service_timeout_sec_);
}

void IndustrialRoutePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up IndustrialRoutePlanner");
  plan_client_.reset();
}

void IndustrialRoutePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating IndustrialRoutePlanner");
}

void IndustrialRoutePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating IndustrialRoutePlanner");
}

nav_msgs::msg::Path IndustrialRoutePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  if (clock_) {
    path.header.stamp = clock_->now();
  }
  path.header.frame_id = global_frame_;

  // Wait for service
  if (!plan_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(logger_, "Route planning service not available: %s", service_name_.c_str());
    return path;  // Return empty path, let fallback planner take over
  }

  // Create request
  auto request = std::make_shared<industrial_route_interfaces::srv::PlanRoute::Request>();
  request->start = start;
  request->goal = goal;
  request->use_start_pose = true;
  request->use_goal_pose = true;

  RCLCPP_INFO(logger_, "Calling industrial_route plan_route service...");

  // Call service synchronously
  auto future = plan_client_->async_send_request(request);
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Node expired");
    return path;
  }

  // Wait for result with timeout
  auto status = future.wait_for(std::chrono::duration<double>(service_timeout_sec_));
  if (status != std::future_status::ready) {
    RCLCPP_WARN(logger_, "Route planning service timed out");
    return path;
  }

  auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(logger_, "Route planning failed: %s", response->error_msg.c_str());
    return path;  // Return empty path for fallback
  }

  // Merge approach + dense + departure paths
  path.header.frame_id = global_frame_;
  if (clock_) {
    path.header.stamp = clock_->now();
  }

  // Add approach path
  for (const auto & pose : response->approach_path.poses) {
    path.poses.push_back(pose);
  }

  // Add dense path
  for (const auto & pose : response->dense_path.poses) {
    path.poses.push_back(pose);
  }

  // Add departure path
  for (const auto & pose : response->departure_path.poses) {
    path.poses.push_back(pose);
  }

  RCLCPP_INFO(logger_, 
    "Route planned: %zu poses (approach: %zu, route: %zu, departure: %zu)",
    path.poses.size(),
    response->approach_path.poses.size(),
    response->dense_path.poses.size(),
    response->departure_path.poses.size());

  return path;
}

}  // namespace industrial_route_ros2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(industrial_route_ros2::IndustrialRoutePlanner, nav2_core::GlobalPlanner)
