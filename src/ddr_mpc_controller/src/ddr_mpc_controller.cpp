#include "ddr_mpc_controller/ddr_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include <Eigen/Sparse>

#ifdef DDR_MPC_HAS_OSQP
#include "osqp/osqp.h"
#endif

namespace ddr_navigation
{
namespace
{
inline Eigen::Vector2d toEigen(const geometry_msgs::msg::PoseStamped & pose)
{
  return Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);
}
}  // namespace

DdrMpcController::DdrMpcController() = default;

void DdrMpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Lifecycle node expired while configuring MPC controller");
  }

  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  global_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();

  horizon_dt_ = node_->declare_parameter(plugin_name_ + ".mpc_dt", horizon_dt_);
  horizon_steps_ = node_->declare_parameter(plugin_name_ + ".horizon_steps", horizon_steps_);
  nominal_speed_ = node_->declare_parameter(plugin_name_ + ".nominal_speed", nominal_speed_);
  min_speed_ = node_->declare_parameter(plugin_name_ + ".min_speed", min_speed_);
  max_speed_ = node_->declare_parameter(plugin_name_ + ".max_speed", max_speed_);
  max_acc_ = node_->declare_parameter(plugin_name_ + ".max_acc", max_acc_);
  max_omega_ = node_->declare_parameter(plugin_name_ + ".max_omega", max_omega_);
  max_domega_ = node_->declare_parameter(plugin_name_ + ".max_domega", max_domega_);
  q_vel_ = node_->declare_parameter(plugin_name_ + ".weight_velocity", q_vel_);
  q_omega_ = node_->declare_parameter(plugin_name_ + ".weight_angular", q_omega_);
  r_vel_ = node_->declare_parameter(plugin_name_ + ".weight_velocity_smooth", r_vel_);
  r_omega_ = node_->declare_parameter(plugin_name_ + ".weight_angular_smooth", r_omega_);
  stopping_distance_ = node_->declare_parameter(plugin_name_ + ".stopping_distance", stopping_distance_);
  goal_lin_tolerance_ = node_->declare_parameter(plugin_name_ + ".goal_lin_tolerance", goal_lin_tolerance_);
  goal_ang_tolerance_ = node_->declare_parameter(plugin_name_ + ".goal_ang_tolerance", goal_ang_tolerance_);

  horizon_steps_ = std::max(2, horizon_steps_);
  if (horizon_dt_ <= 0.0) {
    horizon_dt_ = 0.05;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[%s] configured (dt=%.3f, horizon=%d)", plugin_name_.c_str(), horizon_dt_, horizon_steps_);

  icr_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "EKF_ICR", 1, std::bind(&DdrMpcController::icrCallback, this, std::placeholders::_1));
}

void DdrMpcController::cleanup()
{
  tf_.reset();
  costmap_ros_.reset();
  global_plan_.poses.clear();
  path_lengths_.clear();
  node_.reset();
}

void DdrMpcController::activate() {}

void DdrMpcController::deactivate() {}

void DdrMpcController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  rebuildPathCache();
}

void DdrMpcController::rebuildPathCache()
{
  path_lengths_.clear();
  if (global_plan_.poses.empty()) {
    return;
  }

  path_lengths_.resize(global_plan_.poses.size());
  path_lengths_[0] = 0.0;
  double accum = 0.0;
  for (size_t i = 1; i < global_plan_.poses.size(); ++i) {
    const Eigen::Vector2d prev = toEigen(global_plan_.poses[i - 1]);
    const Eigen::Vector2d curr = toEigen(global_plan_.poses[i]);
    accum += (curr - prev).norm();
    path_lengths_[i] = accum;
  }
}

DdrMpcController::Projection DdrMpcController::findClosestPoint(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  Projection proj{0u, std::numeric_limits<double>::max()};
  if (global_plan_.poses.empty()) {
    return proj;
  }
  const Eigen::Vector2d target = toEigen(pose);
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const Eigen::Vector2d point = toEigen(global_plan_.poses[i]);
    const double dist = (point - target).squaredNorm();
    if (dist < proj.squared_distance) {
      proj.squared_distance = dist;
      proj.index = i;
    }
  }
  return proj;
}

DdrMpcController::ArcPose DdrMpcController::interpolateAtLength(double target_length) const
{
  ArcPose result;
  if (global_plan_.poses.empty() || path_lengths_.empty()) {
    result.position.setZero();
    result.yaw = 0.0;
    result.arc_length = 0.0;
    return result;
  }

  const double clamped = std::clamp(target_length, 0.0, path_lengths_.back());
  auto upper = std::lower_bound(path_lengths_.begin(), path_lengths_.end(), clamped);
  size_t idx = std::distance(path_lengths_.begin(), upper);
  if (idx == 0) {
    result.position = toEigen(global_plan_.poses.front());
    result.yaw = tf2::getYaw(global_plan_.poses.front().pose.orientation);
    result.arc_length = 0.0;
    return result;
  }
  if (idx >= path_lengths_.size()) {
    result.position = toEigen(global_plan_.poses.back());
    result.yaw = tf2::getYaw(global_plan_.poses.back().pose.orientation);
    result.arc_length = path_lengths_.back();
    return result;
  }

  const size_t prev_idx = idx - 1;
  const double span = path_lengths_[idx] - path_lengths_[prev_idx];
  const double ratio = span > 1e-6 ? (clamped - path_lengths_[prev_idx]) / span : 0.0;
  const Eigen::Vector2d start = toEigen(global_plan_.poses[prev_idx]);
  const Eigen::Vector2d end = toEigen(global_plan_.poses[idx]);
  const double yaw_start = tf2::getYaw(global_plan_.poses[prev_idx].pose.orientation);
  const double yaw_end = tf2::getYaw(global_plan_.poses[idx].pose.orientation);

  result.position = start + ratio * (end - start);
  const double yaw_delta = normalizeAngle(yaw_end - yaw_start);
  result.yaw = normalizeAngle(yaw_start + ratio * yaw_delta);
  result.arc_length = clamped;
  return result;
}

DdrMpcController::ReferenceSignals DdrMpcController::buildReferenceSignals(
  double current_arc,
  double current_yaw,
  const Eigen::Vector2d & current_position) const
{
  ReferenceSignals refs;
  refs.linear.assign(horizon_steps_, 0.0);
  refs.angular.assign(horizon_steps_, 0.0);

  if (global_plan_.poses.empty() || path_lengths_.empty()) {
    return refs;
  }

  const double arc_end = path_lengths_.back();
  const double remaining = std::max(0.0, arc_end - current_arc);
  double decel = 1.0;
  if (remaining < stopping_distance_) {
    decel = remaining / std::max(stopping_distance_, 1e-3);
  }
  decel = std::clamp(decel, 0.0, 1.0);

  double prev_yaw = current_yaw;
  Eigen::Vector2d prev_point = current_position;
  const double scaled_speed = nominal_speed_ * speed_limit_scale_;

  for (int k = 0; k < horizon_steps_; ++k) {
    const double travel = scaled_speed * static_cast<double>(k + 1) * horizon_dt_;
    const double target_arc = std::min(arc_end, current_arc + travel);
    const auto ref_pose = interpolateAtLength(target_arc);

    const double distance = (ref_pose.position - prev_point).norm();
    double v = distance / horizon_dt_;
    v *= decel;
    refs.linear[k] = clamp(v, min_speed_, max_speed_ * speed_limit_scale_);

    const double yaw_diff = normalizeAngle(ref_pose.yaw - prev_yaw);
    double w = yaw_diff / horizon_dt_;
    refs.angular[k] = clamp(w, -max_omega_, max_omega_);

    prev_point = ref_pose.position;
    prev_yaw = ref_pose.yaw;
  }

  return refs;
}

bool DdrMpcController::solveVelocityMpc(
  const std::vector<double> & ref_v,
  const std::vector<double> & ref_w,
  double & cmd_v,
  double & cmd_w) const
{
#ifdef DDR_MPC_HAS_OSQP
  if (solveWithOsqp(ref_v, ref_w, cmd_v, cmd_w)) {
    return true;
  }
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[%s] OSQP backend unavailable or failed, using cascade fallback", plugin_name_.c_str());
#endif
  solveWithCascade(ref_v, ref_w, cmd_v, cmd_w);
  return true;
}

void DdrMpcController::solveWithCascade(
  const std::vector<double> & ref_v,
  const std::vector<double> & ref_w,
  double & cmd_v,
  double & cmd_w) const
{
  auto clamp_profile = [this](const std::vector<double> & ref, double min_val,
      double max_val, double max_delta) {
      std::vector<double> out = ref;
      if (out.empty()) {
        return out;
      }
      for (double & val : out) {
        val = clamp(val, min_val, max_val);
      }
      for (size_t i = 1; i < out.size(); ++i) {
        const double upper = out[i - 1] + max_delta;
        const double lower = out[i - 1] - max_delta;
        out[i] = clamp(out[i], lower, upper);
      }
      for (size_t i = out.size(); i-- > 1;) {
        const double upper = out[i] + max_delta;
        const double lower = out[i] - max_delta;
        out[i - 1] = clamp(out[i - 1], lower, upper);
      }
      return out;
    };

  const double max_delta_v = max_acc_ * horizon_dt_;
  const double max_delta_w = max_domega_ * horizon_dt_;
  auto v_profile = clamp_profile(ref_v, min_speed_, max_speed_ * speed_limit_scale_, max_delta_v);
  auto w_profile = clamp_profile(ref_w, -max_omega_, max_omega_, max_delta_w);

  if (!v_profile.empty()) {
    cmd_v = v_profile.front();
  } else {
    cmd_v = 0.0;
  }
  if (!w_profile.empty()) {
    cmd_w = w_profile.front();
  } else {
    cmd_w = 0.0;
  }
}

#ifdef DDR_MPC_HAS_OSQP
namespace
{
struct CscStorage
{
  std::vector<c_float> values;
  std::vector<c_int> row_indices;
  std::vector<c_int> col_ptrs;
};

CscStorage toCsc(const Eigen::SparseMatrix<double> & matrix)
{
  CscStorage storage;
  Eigen::SparseMatrix<double> tmp = matrix;
  tmp.makeCompressed();
  storage.values.resize(tmp.nonZeros());
  storage.row_indices.resize(tmp.nonZeros());
  storage.col_ptrs.resize(tmp.outerSize() + 1);
  for (int k = 0; k < tmp.nonZeros(); ++k) {
    storage.values[k] = static_cast<c_float>(tmp.valuePtr()[k]);
    storage.row_indices[k] = static_cast<c_int>(tmp.innerIndexPtr()[k]);
  }
  for (int i = 0; i < tmp.outerSize() + 1; ++i) {
    storage.col_ptrs[i] = static_cast<c_int>(tmp.outerIndexPtr()[i]);
  }
  return storage;
}
}  // namespace

bool DdrMpcController::solveWithOsqp(
  const std::vector<double> & ref_v,
  const std::vector<double> & ref_w,
  double & cmd_v,
  double & cmd_w) const
{
  const int n = static_cast<int>(ref_v.size());
  if (n == 0) {
    cmd_v = 0.0;
    cmd_w = 0.0;
    return true;
  }
  const int n_vars = 2 * n;
  const int constraint_rows = 2 * n + 4 * (n - 1);

  Eigen::SparseMatrix<double> H(n_vars, n_vars);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(6 * n);
  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(i, i, 2.0 * q_vel_);
    triplets.emplace_back(n + i, n + i, 2.0 * q_omega_);
  }
  for (int i = 1; i < n; ++i) {
    triplets.emplace_back(i - 1, i - 1, 2.0 * r_vel_);
    triplets.emplace_back(i, i, 2.0 * r_vel_);
    triplets.emplace_back(i, i - 1, -2.0 * r_vel_);
    triplets.emplace_back(i - 1, i, -2.0 * r_vel_);

    const int row = n + i;
    triplets.emplace_back(row - 1, row - 1, 2.0 * r_omega_);
    triplets.emplace_back(row, row, 2.0 * r_omega_);
    triplets.emplace_back(row, row - 1, -2.0 * r_omega_);
    triplets.emplace_back(row - 1, row, -2.0 * r_omega_);
  }
  H.setFromTriplets(triplets.begin(), triplets.end());

  Eigen::VectorXd grad(n_vars);
  for (int i = 0; i < n; ++i) {
    grad(i) = -2.0 * q_vel_ * ref_v[i];
    grad(n + i) = -2.0 * q_omega_ * ref_w[i];
  }

  Eigen::SparseMatrix<double> A(constraint_rows, n_vars);
  std::vector<Eigen::Triplet<double>> a_triplets;
  a_triplets.reserve(6 * n);
  Eigen::VectorXd lower(constraint_rows);
  Eigen::VectorXd upper(constraint_rows);

  int row = 0;
  for (int i = 0; i < n; ++i, ++row) {
    a_triplets.emplace_back(row, i, 1.0);
    lower(row) = min_speed_;
    upper(row) = max_speed_ * speed_limit_scale_;
  }
  for (int i = 0; i < n; ++i, ++row) {
    a_triplets.emplace_back(row, n + i, 1.0);
    lower(row) = -max_omega_;
    upper(row) = max_omega_;
  }

  const double max_delta_v = max_acc_ * horizon_dt_;
  const double max_delta_w = max_domega_ * horizon_dt_;
  for (int i = 1; i < n; ++i) {
    a_triplets.emplace_back(row, i, 1.0);
    a_triplets.emplace_back(row, i - 1, -1.0);
    lower(row) = -max_delta_v;
    upper(row) = max_delta_v;
    ++row;
    a_triplets.emplace_back(row, i, -1.0);
    a_triplets.emplace_back(row, i - 1, 1.0);
    lower(row) = -max_delta_v;
    upper(row) = max_delta_v;
    ++row;
  }
  for (int i = 1; i < n; ++i) {
    a_triplets.emplace_back(row, n + i, 1.0);
    a_triplets.emplace_back(row, n + i - 1, -1.0);
    lower(row) = -max_delta_w;
    upper(row) = max_delta_w;
    ++row;
    a_triplets.emplace_back(row, n + i, -1.0);
    a_triplets.emplace_back(row, n + i - 1, 1.0);
    lower(row) = -max_delta_w;
    upper(row) = max_delta_w;
    ++row;
  }

  A.setFromTriplets(a_triplets.begin(), a_triplets.end());

  const auto H_csc = toCsc(H);
  const auto A_csc = toCsc(A);

  std::vector<c_float> q_vec(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    q_vec[i] = static_cast<c_float>(grad(i));
  }
  std::vector<c_float> lower_vec(constraint_rows);
  std::vector<c_float> upper_vec(constraint_rows);
  for (int i = 0; i < constraint_rows; ++i) {
    lower_vec[i] = static_cast<c_float>(lower(i));
    upper_vec[i] = static_cast<c_float>(upper(i));
  }

  OSQPSettings * settings = reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  if (!settings) {
    return false;
  }
  osqp_set_default_settings(settings);
  settings->alpha = 1.6;
  settings->verbose = false;
  settings->eps_abs = 1e-4;
  settings->eps_rel = 1e-4;

  OSQPData * data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));
  if (!data) {
    c_free(settings);
    return false;
  }
  data->n = n_vars;
  data->m = constraint_rows;
  data->P = csc_matrix(n_vars, n_vars, H_csc.values.size(),
      const_cast<c_float *>(H_csc.values.data()),
      const_cast<c_int *>(H_csc.row_indices.data()),
      const_cast<c_int *>(H_csc.col_ptrs.data()));
  data->q = q_vec.data();
  data->A = csc_matrix(constraint_rows, n_vars, A_csc.values.size(),
      const_cast<c_float *>(A_csc.values.data()),
      const_cast<c_int *>(A_csc.row_indices.data()),
      const_cast<c_int *>(A_csc.col_ptrs.data()));
  data->l = lower_vec.data();
  data->u = upper_vec.data();

  OSQPWorkspace * work = nullptr;
  const c_int status = osqp_setup(&work, data, settings);
  if (status != 0) {
    osqp_cleanup(work);
    csc_spfree(data->P);
    csc_spfree(data->A);
    c_free(data);
    c_free(settings);
    return false;
  }

  osqp_solve(work);
  const bool success = work->info->status_val == OSQP_SOLVED ||
    work->info->status_val == OSQP_SOLVED_INACCURATE;
  if (success) {
    cmd_v = work->solution->x[0];
    cmd_w = work->solution->x[n];
  }

  osqp_cleanup(work);
  csc_spfree(data->P);
  csc_spfree(data->A);
  c_free(data);
  c_free(settings);
  return success;
}
#endif

double DdrMpcController::normalizeAngle(double angle) const
{
  double a = angle;
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

double DdrMpcController::clamp(double value, double low, double high) const
{
  return std::min(std::max(value, low), high);
}

geometry_msgs::msg::TwistStamped DdrMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (!node_) {
    throw std::runtime_error("MPC controller is not configured");
  }
  if (global_plan_.poses.empty() || path_lengths_.empty()) {
    throw std::runtime_error("MPC controller received empty plan");
  }

  const auto goal_pose = global_plan_.poses.back();
  const bool reached = goal_checker &&
    goal_checker->isGoalReached(pose.pose, goal_pose.pose, velocity);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_frame_;
  cmd.header.stamp = node_->now();

  if (reached) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    last_cmd_v_ = 0.0;
    last_cmd_w_ = 0.0;
    return cmd;
  }

  const auto projection = findClosestPoint(pose);
  const double current_arc = path_lengths_[projection.index];
  const Eigen::Vector2d robot_xy = toEigen(pose);
  const double robot_yaw = tf2::getYaw(pose.pose.orientation);

  const auto refs = buildReferenceSignals(current_arc, robot_yaw, robot_xy);
  double cmd_v = 0.0;
  double cmd_w = 0.0;
  solveVelocityMpc(refs.linear, refs.angular, cmd_v, cmd_w);

  const double distance_to_goal = path_lengths_.back() - current_arc;
  if (distance_to_goal < goal_lin_tolerance_) {
    cmd_v = 0.0;
    const double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
    const double yaw_error = normalizeAngle(goal_yaw - robot_yaw);
    if (std::fabs(yaw_error) < goal_ang_tolerance_) {
      cmd_w = 0.0;
    }
  }

  cmd.twist.linear.x = cmd_v;
  cmd.twist.angular.z = cmd_w;
  last_cmd_v_ = cmd_v;
  last_cmd_w_ = cmd_w;
  return cmd;
}

void DdrMpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    speed_limit_scale_ = 1.0;
    return;
  }
  if (percentage) {
    speed_limit_scale_ = clamp(speed_limit / 100.0, 0.0, 1.0);
  } else {
    speed_limit_scale_ = clamp(speed_limit / max_speed_, 0.0, 1.0);
  }
}

void DdrMpcController::icrCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  car_icr_.yr = msg->point.x;
  car_icr_.yl = msg->point.y;
  car_icr_.xv = msg->point.z;
}

}  // namespace ddr_navigation

PLUGINLIB_EXPORT_CLASS(ddr_navigation::DdrMpcController, nav2_core::Controller)
