#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace ddr_navigation
{
class FirstOrderFilter
{
public:
  FirstOrderFilter(double cutoff, double sampling)
  : a_(std::exp(-2.0 * M_PI * cutoff / sampling)),
    b_(1.0 - a_),
    prev_output_(0.0)
  {}

  double filter(double input)
  {
    const double output = b_ * input + a_ * prev_output_;
    prev_output_ = output;
    return output;
  }

private:
  double a_;
  double b_;
  double prev_output_;
};

class ICREKFNode : public rclcpp::Node
{
public:
  ICREKFNode();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publishState();

  void getForecast(Eigen::VectorXd & x, Eigen::MatrixXd & conv,
    const Eigen::Vector2d & input_u, double duration);
  void getUpdate(Eigen::VectorXd & x, Eigen::MatrixXd & conv,
    const Eigen::Vector3d & current_state);

  // Parameters / configuration
  int pose_reduce_frequency_;
  int pose_reduce_count_{0};
  double state_pub_frequency_;
  double wheel_track_;
  double yr_standard_;
  double yl_standard_;
  double xv_standard_;
  bool if_update_;
  std::string odom_topic_;
  std::string cmd_topic_;
  std::string state_topic_;
  std::string icr_topic_;
  std::string simple_icr_topic_;
  std::string eigenvalues_topic_;
  std::string eigenvalues_all_topic_;

  // EKF state
  bool have_state_{false};
  bool have_input_{false};
  Eigen::Vector3d current_state_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_state_v_{Eigen::Vector3d::Zero()};
  double current_state_omega_{0.0};
  rclcpp::Time current_time_;
  rclcpp::Time current_u_time_;
  double pre_u_duration_{0.0};
  Eigen::Vector2d current_u_{Eigen::Vector2d::Zero()};

  Eigen::VectorXd x_;
  Eigen::MatrixXd cov_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd L_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd M_;
  Eigen::MatrixXd R_;

  bool if_yr_conver_{false};
  bool if_yl_conver_{false};
  bool if_xv_conver_{false};
  int index_yr_standard_{0};
  int index_yl_standard_{0};
  int index_xv_standard_{0};
  double start_time_{-1.0};

  std::shared_ptr<FirstOrderFilter> filter_yl_;
  std::shared_ptr<FirstOrderFilter> filter_yr_;
  std::shared_ptr<FirstOrderFilter> filter_xv_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr icr_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr simple_icr_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr eigenvalues_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr eigenvalues_all_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};
}  // namespace ddr_navigation
