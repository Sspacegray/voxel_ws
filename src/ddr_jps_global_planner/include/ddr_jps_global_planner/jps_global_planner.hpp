#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

#include "ddr_jps_global_planner/costmap_adapter.hpp"
#include "ddr_jps_global_planner/graph_search.hpp"

namespace ddr_navigation
{
class DdrJpsGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  DdrJpsGlobalPlanner();
  ~DdrJpsGlobalPlanner() override = default;

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
  bool transformToGlobalFrame(
    const geometry_msgs::msg::PoseStamped & in,
    geometry_msgs::msg::PoseStamped & out) const;

  std::vector<Eigen::Vector2d> convertStatesToWorld(const std::vector<JPS::StatePtr> & states) const;
  std::vector<Eigen::Vector2d> simplifyPath(const std::vector<Eigen::Vector2d> & raw_path) const;
  nav_msgs::msg::Path composePathMsg(
    const std::vector<Eigen::Vector2d> & points,
    const rclcpp::Time & stamp) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<CostmapAdapter> costmap_adapter_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

  double safe_distance_ = 0.3;
  bool use_jps_ = true;
  int max_iterations_ = -1;
  double goal_tolerance_ = 0.05;
  bool unknown_is_free_ = false;
  double transform_tolerance_ = 0.2;
};
}  // namespace ddr_navigation
