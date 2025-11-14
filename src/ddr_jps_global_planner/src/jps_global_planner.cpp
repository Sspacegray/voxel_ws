#include "ddr_jps_global_planner/jps_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ddr_navigation
{
namespace
{
inline Eigen::Vector2d toEigen(const geometry_msgs::msg::PoseStamped & pose)
{
  return Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);
}
}

DdrJpsGlobalPlanner::DdrJpsGlobalPlanner() = default;

void DdrJpsGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  if (!parent.lock()) {
    throw nav2_core::PlannerException("Lifecycle node expired while configuring JPS planner");
  }
  node_ = parent.lock();
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  global_frame_ = costmap_ros_->getGlobalFrameID();

  safe_distance_ = node_->declare_parameter(plugin_name_ + ".safe_distance", safe_distance_);
  use_jps_ = node_->declare_parameter(plugin_name_ + ".use_jps", use_jps_);
  max_iterations_ = node_->declare_parameter(plugin_name_ + ".max_iterations", max_iterations_);
  goal_tolerance_ = node_->declare_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);
  unknown_is_free_ = node_->declare_parameter(plugin_name_ + ".unknown_is_free", unknown_is_free_);
  transform_tolerance_ = node_->declare_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);

  costmap_adapter_ = std::make_shared<CostmapAdapter>(costmap_ros_->getCostmap(), unknown_is_free_);
  costmap_adapter_->updateDistanceField();

  plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + std::string("/plan"), rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(node_->get_logger(), "[%s] configured", plugin_name_.c_str());
}

void DdrJpsGlobalPlanner::cleanup()
{
  if (plan_pub_) {
    plan_pub_.reset();
  }
  costmap_adapter_.reset();
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
}

void DdrJpsGlobalPlanner::activate()
{
  if (plan_pub_) {
    plan_pub_->on_activate();
  }
}

void DdrJpsGlobalPlanner::deactivate()
{
  if (plan_pub_) {
    plan_pub_->on_deactivate();
  }
}

nav_msgs::msg::Path DdrJpsGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!node_) {
    throw nav2_core::PlannerException("JPS planner is not configured");
  }

  geometry_msgs::msg::PoseStamped global_start;
  geometry_msgs::msg::PoseStamped global_goal;
  if (!transformToGlobalFrame(start, global_start) || !transformToGlobalFrame(goal, global_goal)) {
    throw nav2_core::PlannerException("Failed to transform poses into global frame");
  }

  const Eigen::Vector2d start_xy = toEigen(global_start);
  const Eigen::Vector2d goal_xy = toEigen(global_goal);
  const double goal_distance = (goal_xy - start_xy).norm();
  if (goal_distance < goal_tolerance_) {
    nav_msgs::msg::Path trivial;
    trivial.header.frame_id = global_frame_;
    trivial.header.stamp = global_start.header.stamp;
    trivial.poses = {global_start, global_goal};
    if (plan_pub_ && plan_pub_->is_activated()) {
      plan_pub_->publish(trivial);
    }
    return trivial;
  }

  costmap_adapter_->setCostmap(costmap_ros_->getCostmap());
  costmap_adapter_->updateDistanceField();

  const auto start_idx = costmap_adapter_->coord2gridIndex(start_xy);
  const auto goal_idx = costmap_adapter_->coord2gridIndex(goal_xy);
  if (start_idx.x() < 0 || start_idx.y() < 0) {
    throw nav2_core::PlannerException("Start pose is outside of the costmap bounds");
  }
  if (goal_idx.x() < 0 || goal_idx.y() < 0) {
    throw nav2_core::PlannerException("Goal pose is outside of the costmap bounds");
  }

  auto graph = std::make_unique<JPS::GraphSearch>(costmap_adapter_, safe_distance_);
  graph->SetSafeDis(safe_distance_);
  if (!graph->plan(start_idx.x(), start_idx.y(), goal_idx.x(), goal_idx.y(), use_jps_, max_iterations_)) {
    throw nav2_core::PlannerException("Failed to generate JPS path");
  }

  auto states = graph->getPath();
  auto raw_points = convertStatesToWorld(states);
  if (raw_points.empty()) {
    throw nav2_core::PlannerException("Graph search returned empty path");
  }
  raw_points.front() = start_xy;
  raw_points.back() = goal_xy;

  auto simplified = simplifyPath(raw_points);
  auto path_msg = composePathMsg(simplified, global_start.header.stamp);

  if (plan_pub_ && plan_pub_->is_activated()) {
    plan_pub_->publish(path_msg);
  }

  return path_msg;
}

bool DdrJpsGlobalPlanner::transformToGlobalFrame(
  const geometry_msgs::msg::PoseStamped & in,
  geometry_msgs::msg::PoseStamped & out) const
{
  if (in.header.frame_id.empty()) {
    return false;
  }

  if (in.header.frame_id == global_frame_) {
    out = in;
    return true;
  }

  try {
    tf_->transform(in, out, global_frame_, tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException & ex) {
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "[%s] TF transform failed: %s", plugin_name_.c_str(), ex.what());
    }
    return false;
  }
}

std::vector<Eigen::Vector2d> DdrJpsGlobalPlanner::convertStatesToWorld(
  const std::vector<JPS::StatePtr> & states) const
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(states.size());
  for (const auto & state : states) {
    points.emplace_back(costmap_adapter_->gridIndex2coordd(Eigen::Vector2i(state->x, state->y)));
  }
  std::reverse(points.begin(), points.end());
  return points;
}

std::vector<Eigen::Vector2d> DdrJpsGlobalPlanner::simplifyPath(
  const std::vector<Eigen::Vector2d> & raw_path) const
{
  if (raw_path.size() <= 2) {
    return raw_path;
  }

  std::vector<Eigen::Vector2d> simplified;
  simplified.reserve(raw_path.size());
  simplified.push_back(raw_path.front());

  Eigen::Vector2d prev_dir = raw_path[1] - raw_path[0];
  if (prev_dir.norm() > 1e-6) {
    prev_dir.normalize();
  }

  for (size_t i = 1; i < raw_path.size() - 1; ++i) {
    Eigen::Vector2d next_dir = raw_path[i + 1] - raw_path[i];
    if (next_dir.norm() < 1e-6) {
      continue;
    }
    next_dir.normalize();
    if ((next_dir - prev_dir).norm() > 1e-3) {
      simplified.push_back(raw_path[i]);
      prev_dir = next_dir;
    }
  }

  simplified.push_back(raw_path.back());
  return simplified;
}

nav_msgs::msg::Path DdrJpsGlobalPlanner::composePathMsg(
  const std::vector<Eigen::Vector2d> & points,
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = global_frame_;
  if (stamp.nanoseconds() == 0 && node_) {
    msg.header.stamp = node_->now();
  } else {
    msg.header.stamp = stamp;
  }

  if (points.empty()) {
    return msg;
  }

  msg.poses.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = points[i].x();
    pose.pose.position.y = points[i].y();
    pose.pose.position.z = 0.0;

    double yaw = 0.0;
    if (i < points.size() - 1) {
      const auto dir = points[i + 1] - points[i];
      yaw = std::atan2(dir.y(), dir.x());
    } else if (i > 0) {
      const auto dir = points[i] - points[i - 1];
      yaw = std::atan2(dir.y(), dir.x());
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    msg.poses.push_back(std::move(pose));
  }

  return msg;
}
}  // namespace ddr_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ddr_navigation::DdrJpsGlobalPlanner, nav2_core::GlobalPlanner)
