#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ddr_navigation
{
class DdrTrajectoryOptimizer : public nav2_core::Smoother
{
public:
  DdrTrajectoryOptimizer();
  ~DdrTrajectoryOptimizer() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

protected:
  using Path2D = std::vector<Eigen::Vector2d>;

  Path2D toEigenPath(const nav_msgs::msg::Path & path) const;
  void toMsgPath(const Path2D & points, nav_msgs::msg::Path & path) const;
  void runSmoothing(Path2D & working_path, const Path2D & reference_path, const rclcpp::Duration & max_time);
  Path2D resamplePath(const Path2D & path) const;
  double computeCurvature(const Eigen::Vector2d & p0, const Eigen::Vector2d & p1, const Eigen::Vector2d & p2) const;
  bool isCollisionFree(const Eigen::Vector2d & pt) const;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;

  double data_weight_ = 0.25;
  double smooth_weight_ = 0.4;
  double curvature_weight_ = 0.1;
  double max_curvature_ = 0.6;  // 1/m
  double sampling_resolution_ = 0.05;
  int max_iterations_ = 200;
  double tolerance_ = 1e-3;
  double collision_cost_threshold_ = 253.0;
};
}  // namespace ddr_navigation
