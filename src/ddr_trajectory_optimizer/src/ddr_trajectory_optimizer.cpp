#include "ddr_trajectory_optimizer/ddr_trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
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

DdrTrajectoryOptimizer::DdrTrajectoryOptimizer() = default;

void DdrTrajectoryOptimizer::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Lifecycle node expired while configuring trajectory optimizer");
  }

  plugin_name_ = std::move(name);
  costmap_sub_ = std::move(costmap_sub);
  footprint_sub_ = std::move(footprint_sub);

  data_weight_ = node_->declare_parameter(plugin_name_ + ".data_weight", data_weight_);
  smooth_weight_ = node_->declare_parameter(plugin_name_ + ".smooth_weight", smooth_weight_);
  curvature_weight_ = node_->declare_parameter(plugin_name_ + ".curvature_weight", curvature_weight_);
  max_curvature_ = node_->declare_parameter(plugin_name_ + ".max_curvature", max_curvature_);
  sampling_resolution_ = node_->declare_parameter(plugin_name_ + ".sampling_resolution", sampling_resolution_);
  max_iterations_ = node_->declare_parameter(plugin_name_ + ".max_iterations", max_iterations_);
  tolerance_ = node_->declare_parameter(plugin_name_ + ".tolerance", tolerance_);
  collision_cost_threshold_ = node_->declare_parameter(
    plugin_name_ + ".collision_cost_threshold",
    static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE));

  RCLCPP_INFO(node_->get_logger(), "[%s] configured", plugin_name_.c_str());
}

void DdrTrajectoryOptimizer::cleanup()
{
  costmap_sub_.reset();
  footprint_sub_.reset();
  node_.reset();
}

void DdrTrajectoryOptimizer::activate() {}

void DdrTrajectoryOptimizer::deactivate() {}

bool DdrTrajectoryOptimizer::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  if (!node_) {
    throw std::runtime_error("Trajectory optimizer is not configured");
  }

  if (path.poses.size() < 3) {
    return true;
  }

  const auto reference_path = toEigenPath(path);
  auto working_path = reference_path;
  runSmoothing(working_path, reference_path, max_time);
  auto resampled = resamplePath(working_path);
  toMsgPath(resampled, path);
  return true;
}

DdrTrajectoryOptimizer::Path2D DdrTrajectoryOptimizer::toEigenPath(const nav_msgs::msg::Path & path) const
{
  Path2D result;
  result.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    result.emplace_back(toEigen(pose));
  }
  return result;
}

void DdrTrajectoryOptimizer::toMsgPath(const Path2D & points, nav_msgs::msg::Path & path) const
{
  path.poses.clear();
  path.poses.reserve(points.size());
  path.header.stamp = node_ ? node_->now() : rclcpp::Time{};

  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[i].x();
    pose.pose.position.y = points[i].y();
    pose.pose.position.z = 0.0;

    double yaw = 0.0;
    if (i + 1 < points.size()) {
      Eigen::Vector2d delta = points[i + 1] - points[i];
      if (delta.norm() > 1e-6) {
        yaw = std::atan2(delta.y(), delta.x());
      } else if (i > 0) {
        delta = points[i] - points[i - 1];
        if (delta.norm() > 1e-6) {
          yaw = std::atan2(delta.y(), delta.x());
        }
      }
    } else if (i > 0) {
      const Eigen::Vector2d delta = points[i] - points[i - 1];
      if (delta.norm() > 1e-6) {
        yaw = std::atan2(delta.y(), delta.x());
      }
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(std::move(pose));
  }
}

void DdrTrajectoryOptimizer::runSmoothing(
  Path2D & working_path,
  const Path2D & reference_path,
  const rclcpp::Duration & max_time)
{
  if (working_path.size() < 3) {
    return;
  }

  const auto start_time = node_->now();
  for (int iter = 0; iter < max_iterations_; ++iter) {
    double max_update = 0.0;
    for (size_t i = 1; i + 1 < working_path.size(); ++i) {
      const auto prev = working_path[i - 1];
      const auto next = working_path[i + 1];
      auto curr = working_path[i];

      Eigen::Vector2d data_term = reference_path[i] - curr;
      Eigen::Vector2d smooth_term = prev + next - 2.0 * curr;

      Eigen::Vector2d curvature_term = Eigen::Vector2d::Zero();
      const double curvature = computeCurvature(prev, curr, next);
      if (curvature > max_curvature_) {
        Eigen::Vector2d tangent = next - prev;
        if (tangent.norm() > 1e-6) {
          tangent.normalize();
          Eigen::Vector2d normal(-tangent.y(), tangent.x());
          curvature_term = - (curvature - max_curvature_) * normal;
        }
      }

      Eigen::Vector2d update = data_weight_ * data_term + smooth_weight_ * smooth_term +
        curvature_weight_ * curvature_term;

      const Eigen::Vector2d candidate = curr + update;
      if (isCollisionFree(candidate)) {
        working_path[i] = candidate;
        max_update = std::max(max_update, update.norm());
      }
    }

    if (max_update < tolerance_) {
      break;
    }

    if ((node_->now() - start_time) > max_time) {
      break;
    }
  }
}

DdrTrajectoryOptimizer::Path2D DdrTrajectoryOptimizer::resamplePath(const Path2D & path) const
{
  if (path.size() < 2 || sampling_resolution_ <= 0.0) {
    return path;
  }

  Path2D resampled;
  resampled.reserve(path.size());
  resampled.push_back(path.front());

  for (size_t i = 1; i < path.size(); ++i) {
    Eigen::Vector2d start = path[i - 1];
    Eigen::Vector2d end = path[i];
    Eigen::Vector2d delta = end - start;
    const double seg_length = delta.norm();
    if (seg_length < 1e-6) {
      continue;
    }

    const int num_steps = static_cast<int>(std::floor(seg_length / sampling_resolution_));
    for (int step = 1; step < num_steps; ++step) {
      const double ratio = (step * sampling_resolution_) / seg_length;
      const Eigen::Vector2d candidate = start + ratio * delta;
      if ((candidate - resampled.back()).norm() > 1e-6 && isCollisionFree(candidate)) {
        resampled.push_back(candidate);
      }
    }

    if ((end - resampled.back()).norm() > 1e-6) {
      resampled.push_back(end);
    }
  }

  resampled.back() = path.back();
  return resampled;
}

double DdrTrajectoryOptimizer::computeCurvature(
  const Eigen::Vector2d & p0,
  const Eigen::Vector2d & p1,
  const Eigen::Vector2d & p2) const
{
  const double a = (p1 - p0).norm();
  const double b = (p2 - p1).norm();
  const double c = (p2 - p0).norm();
  if (a < 1e-6 || b < 1e-6 || c < 1e-6) {
    return 0.0;
  }

  const double area = std::abs(
    p0.x() * (p1.y() - p2.y()) +
    p1.x() * (p2.y() - p0.y()) +
    p2.x() * (p0.y() - p1.y())) * 0.5;
  if (area < 1e-9) {
    return 0.0;
  }
  return 4.0 * area / (a * b * c);
}

bool DdrTrajectoryOptimizer::isCollisionFree(const Eigen::Vector2d & pt) const
{
  if (!costmap_sub_) {
    return true;
  }
  auto costmap = costmap_sub_->getCostmap();
  if (!costmap) {
    return true;
  }
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap->worldToMap(pt.x(), pt.y(), mx, my)) {
    return false;
  }
  const auto cost = costmap->getCost(mx, my);
  return static_cast<double>(cost) < collision_cost_threshold_;
}

}  // namespace ddr_navigation

PLUGINLIB_EXPORT_CLASS(ddr_navigation::DdrTrajectoryOptimizer, nav2_core::Smoother)
