#include "ddr_icrekf/icrekf.hpp"

#include <chrono>

#include "tf2/LinearMath/Quaternion.h"

namespace ddr_navigation
{

ICREKFNode::ICREKFNode()
: rclcpp::Node("ddr_icrekf")
{
  pose_reduce_frequency_ = this->declare_parameter<int>("pose_sub_reduce_frequency", 10);
  if (pose_reduce_frequency_ < 1) {
    pose_reduce_frequency_ = 1;
  }
  state_pub_frequency_ = this->declare_parameter<double>("state_pub_frequency", 10.0);
  if (state_pub_frequency_ <= 0.0) {
    state_pub_frequency_ = 10.0;
  }
  wheel_track_ = this->declare_parameter<double>("wheel_track", 0.4);
  yr_standard_ = this->declare_parameter<double>("yr_standard", 0.15);
  yl_standard_ = this->declare_parameter<double>("yl_standard", 0.15);
  xv_standard_ = this->declare_parameter<double>("xv_standard", 0.0);
  if_update_ = this->declare_parameter<bool>("if_update", true);

  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
  cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");
  state_topic_ = this->declare_parameter<std::string>("state_topic", "ekf_state");
  icr_topic_ = this->declare_parameter<std::string>("icr_topic", "ekf_icr");
  simple_icr_topic_ = this->declare_parameter<std::string>("simple_icr_topic", "simple_ekf_icr");
  eigenvalues_topic_ = this->declare_parameter<std::string>("eigenvalues_topic", "ekf_icr_eigenvalues");
  eigenvalues_all_topic_ = this->declare_parameter<std::string>("eigenvalues_all_topic", "ekf_icr_cov_diag");

  Q_.setZero(6, 6);
  Q_(0, 0) = this->declare_parameter<double>("Q_x", 0.2);
  Q_(1, 1) = this->declare_parameter<double>("Q_y", 0.2);
  Q_(2, 2) = this->declare_parameter<double>("Q_psi", 0.314);
  Q_(3, 3) = this->declare_parameter<double>("Q_yr", 0.01);
  Q_(4, 4) = this->declare_parameter<double>("Q_yl", 0.01);
  Q_(5, 5) = this->declare_parameter<double>("Q_xv", 0.01);
  Q_ = Q_ * Q_;

  R_.setZero(3, 3);
  R_(0, 0) = this->declare_parameter<double>("R_x", 0.01);
  R_(1, 1) = this->declare_parameter<double>("R_y", 0.01);
  R_(2, 2) = this->declare_parameter<double>("R_psi", 0.0157);
  R_ = R_ * R_;

  L_.setIdentity(6, 6);
  H_.setZero(3, 6);
  H_.block(0, 0, 3, 3).setIdentity();
  M_.setIdentity(3, 3);

  x_.setZero(6);
  x_[3] = this->declare_parameter<double>("init_x_yr", 0.01);
  x_[4] = this->declare_parameter<double>("init_x_yl", 0.01);
  x_[5] = this->declare_parameter<double>("init_x_xv", 0.01);
  cov_.setZero(6, 6);

  const double sampling = state_pub_frequency_;
  filter_yl_ = std::make_shared<FirstOrderFilter>(1.0, sampling);
  filter_yr_ = std::make_shared<FirstOrderFilter>(1.0, sampling);
  filter_xv_ = std::make_shared<FirstOrderFilter>(1.0, sampling);

  auto qos = rclcpp::SensorDataQoS();
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, qos, std::bind(&ICREKFNode::odomCallback, this, std::placeholders::_1));
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_topic_, rclcpp::QoS(50),
    std::bind(&ICREKFNode::cmdCallback, this, std::placeholders::_1));

  state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(state_topic_, 10);
  icr_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(icr_topic_, 10);
  simple_icr_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(simple_icr_topic_, 10);
  eigenvalues_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(eigenvalues_topic_, 10);
  eigenvalues_all_pub_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>(eigenvalues_all_topic_, 10);

  const auto period = std::chrono::duration<double>(1.0 / state_pub_frequency_);
  state_timer_ = this->create_wall_timer(
    period, std::bind(&ICREKFNode::publishState, this));

  RCLCPP_INFO(this->get_logger(), "ICREKF node initialized (odom: %s, cmd: %s)",
    odom_topic_.c_str(), cmd_topic_.c_str());
}

void ICREKFNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!if_update_) {
    return;
  }

  const double yaw = tf2::getYaw(msg->pose.pose.orientation);
  current_state_ << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;
  current_state_v_ << msg->twist.twist.linear.x,
    msg->twist.twist.linear.y, msg->twist.twist.angular.z;
  current_state_omega_ = msg->twist.twist.angular.z;
  current_time_ = msg->header.stamp;

  if (!have_state_) {
    x_.head(3) = current_state_;
    have_state_ = true;
    return;
  }

  if (pose_reduce_count_ < pose_reduce_frequency_) {
    ++pose_reduce_count_;
    return;
  }
  pose_reduce_count_ -= pose_reduce_frequency_;

  while (current_state_[2] - x_[2] > M_PI) {
    current_state_[2] -= 2.0 * M_PI;
  }
  while (current_state_[2] - x_[2] < -M_PI) {
    current_state_[2] += 2.0 * M_PI;
  }

  getUpdate(x_, cov_, current_state_);
}

void ICREKFNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!if_update_) {
    return;
  }

  const double vl = msg->linear.x - 0.5 * wheel_track_ * msg->angular.z;
  const double vr = msg->linear.x + 0.5 * wheel_track_ * msg->angular.z;

  if (!have_input_) {
    current_u_ << vl, vr;
    current_u_time_ = this->now();
    have_input_ = true;
    return;
  }

  if (!have_state_) {
    return;
  }

  const double dt = (this->now() - current_u_time_).seconds();
  getForecast(x_, cov_, current_u_, dt);
  current_u_ << vl, vr;
  current_u_time_ = this->now();

  if (start_time_ < 0.0 && current_u_.squaredNorm() > 0.1) {
    start_time_ = this->now().seconds();
  }
}

void ICREKFNode::getForecast(
  Eigen::VectorXd & x,
  Eigen::MatrixXd & conv,
  const Eigen::Vector2d & input_u,
  double duration)
{
  if (!if_update_ || duration <= 0.0) {
    return;
  }

  const double x_pos = x[0];
  const double y_pos = x[1];
  const double psi = x[2];
  const double yr = x[3];
  const double yl = x[4];
  const double xv = x[5];

  const double vl = input_u.x();
  const double vr = input_u.y();

  const double denom = yl - yr;
  if (std::abs(denom) < 1e-6) {
    return;
  }

  x[0] = x_pos + duration * (((vr * yl - vl * yr) / denom) * std::cos(psi) +
    (vr - vl) * xv / denom * std::sin(psi));
  x[1] = y_pos + duration * (((vr * yl - vl * yr) / denom) * std::sin(psi) -
    (vr - vl) * xv / denom * std::cos(psi));
  x[2] = psi + duration * (vr - vl) / denom;

  Eigen::MatrixXd F(6, 6);
  F.setIdentity();
  F(0, 2) =  duration * (-(vr * yl - vl * yr) / denom * std::sin(psi) +
      (vr - vl) * xv / denom * std::cos(psi));
  F(1, 2) =  duration * ((vr * yl - vl * yr) / denom * std::cos(psi) +
      (vr - vl) * xv / denom * std::sin(psi));
  F(2, 3) =  duration * (vr - vl) / (denom * denom);
  F(2, 4) = -duration * (vr - vl) / (denom * denom);

  conv = F.transpose() * conv * F + L_ * duration * Q_ * L_.transpose() * duration;
}

void ICREKFNode::getUpdate(
  Eigen::VectorXd & x,
  Eigen::MatrixXd & conv,
  const Eigen::Vector3d & current_state)
{
  if (!if_update_) {
    return;
  }
  const Eigen::MatrixXd S = H_ * conv * H_.transpose() + M_ * R_ * M_.transpose();
  const Eigen::MatrixXd K = conv * H_.transpose() * S.inverse();
  x = x + K * (current_state - H_ * x);
  conv = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * conv;
}

void ICREKFNode::publishState()
{
  if (!if_update_) {
    return;
  }

  const auto now = this->now();
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = now;
  odom.pose.pose.position.x = x_[0];
  odom.pose.pose.position.y = x_[1];
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, x_[2]);
  odom.pose.pose.orientation = tf2::toMsg(q);
  state_pub_->publish(odom);

  geometry_msgs::msg::PointStamped ps;
  ps.header.stamp = now;
  ps.header.frame_id = "base_link";
  ps.point.x = x_[3];
  ps.point.y = x_[4];
  ps.point.z = x_[5];
  icr_pub_->publish(ps);

  geometry_msgs::msg::PointStamped diag_msg;
  diag_msg.header = ps.header;
  diag_msg.point.x = cov_(0, 0);
  diag_msg.point.y = cov_(1, 1);
  diag_msg.point.z = cov_(2, 2);
  eigenvalues_pub_->publish(diag_msg);

  diag_msg.point.x = cov_(3, 3);
  diag_msg.point.y = cov_(4, 4);
  diag_msg.point.z = cov_(5, 5);
  eigenvalues_all_pub_->publish(diag_msg);

  if (have_state_ && have_input_) {
    geometry_msgs::msg::PointStamped simple_msg;
    simple_msg.header = ps.header;
    if (std::abs(current_state_omega_) > 1e-1) {
      simple_msg.point.x = filter_yl_->filter(
        (current_state_v_.y() - current_u_.x()) / current_state_omega_);
      simple_msg.point.y = filter_yr_->filter(
        (current_state_v_.y() - current_u_.y()) / current_state_omega_);
      simple_msg.point.z = filter_xv_->filter(
        -current_state_v_.z() / current_state_omega_);
    } else {
      simple_msg.point.x = 0.0;
      simple_msg.point.y = 0.0;
      simple_msg.point.z = 0.0;
    }
    simple_icr_pub_->publish(simple_msg);
  }

  auto check_convergence = [&](double value, double target, bool & flag, int & index,
      const std::string & label) {
      if (flag) {
        return;
      }
      if (std::abs(target) < 1e-6) {
        return;
      }
      if (std::abs(value - target) / std::abs(target) < 0.01) {
        if (++index > 10) {
          flag = true;
          RCLCPP_INFO(this->get_logger(), "%s converged in %.2f s",
            label.c_str(), now.seconds() - start_time_);
        }
      } else {
        index = 0;
      }
    };

  if (start_time_ > 0.0) {
    check_convergence(x_[3], yr_standard_, if_yr_conver_, index_yr_standard_, "alpha_l");
    check_convergence(x_[4], yl_standard_, if_yl_conver_, index_yl_standard_, "alpha_r");
    if (std::abs(xv_standard_) > 1e-6) {
      check_convergence(x_[5], xv_standard_, if_xv_conver_, index_xv_standard_, "xv");
    }
  }
}

}  // namespace ddr_navigation
