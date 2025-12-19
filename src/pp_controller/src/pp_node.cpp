#include "pp_controller/pp_node.hpp"
#include "pp_controller/math_utils.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace pp_controller
{

PPNode::PPNode(const rclcpp::NodeOptions& options)
    : Node("pp_controller", options)
    , state_(ControllerState::IDLE)
    , current_segment_idx_(0)
    , target_yaw_(0.0)
    , is_paused_(false)
    , obstacle_detected_(false)
    , lateral_deviation_exceeded_(false)
    , current_lateral_dev_(0.0)
    , current_x_(0.0)
    , current_y_(0.0)
    , current_yaw_(0.0)
    , current_vel_(0.0)
    , pose_valid_(false)
    , last_pose_time_(0, 0, this->get_clock()->get_clock_type())
{
    // === 位姿源参数 ===
    this->declare_parameter("pose_source", std::string("tf"));           // tf | odom_topic
    this->declare_parameter("pose_topic", std::string("/Odometry"));     // nav_msgs/Odometry, in map frame
    this->declare_parameter("pose_timeout", 0.5);                        // seconds
    this->declare_parameter("global_frame", std::string("map"));
    this->declare_parameter("base_frame", std::string("base_link"));
    this->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));

    // === 核心参数 ===
    this->declare_parameter("max_linear_vel", 0.5);
    this->declare_parameter("min_linear_vel", 0.05);
    this->declare_parameter("max_angular_vel", 1.0);
    this->declare_parameter("lookahead_gain", 0.5);
    this->declare_parameter("lookahead_min", 0.2);
    this->declare_parameter("lookahead_max", 1.0);
    this->declare_parameter("position_tolerance", 0.05);
    this->declare_parameter("angle_tolerance", 0.05);
    this->declare_parameter("spin_kp", 2.0);
    this->declare_parameter("control_rate", 20.0);
    this->declare_parameter("path_file", std::string(""));
    
    // === RPP 增强参数 ===
    this->declare_parameter("curvature_min_radius", 0.9);
    this->declare_parameter("max_lateral_deviation", 0.3);
    this->declare_parameter("use_curvature_regulation", true);
    
    // === 障碍物检测参数 ===
    this->declare_parameter("laser_topic", std::string("/scan"));
    this->declare_parameter("obstacle_distance_threshold", 0.5);
    this->declare_parameter("obstacle_angle_range", 0.5);
    
    // === Costmap 参数 ===
    this->declare_parameter("costmap_topic", std::string("/local_costmap/costmap"));
    this->declare_parameter("cost_scaling_dist", 0.6);
    this->declare_parameter("cost_scaling_gain", 0.5);
    this->declare_parameter("use_costmap", true);

    // 获取参数
    pose_source_ = this->get_parameter("pose_source").as_string();
    pose_topic_ = this->get_parameter("pose_topic").as_string();
    pose_timeout_ = this->get_parameter("pose_timeout").as_double();
    global_frame_ = this->get_parameter("global_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();

    max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    lookahead_gain_ = this->get_parameter("lookahead_gain").as_double();
    lookahead_min_ = this->get_parameter("lookahead_min").as_double();
    lookahead_max_ = this->get_parameter("lookahead_max").as_double();
    position_tolerance_ = this->get_parameter("position_tolerance").as_double();
    angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
    spin_kp_ = this->get_parameter("spin_kp").as_double();
    control_rate_ = this->get_parameter("control_rate").as_double();
    path_file_ = this->get_parameter("path_file").as_string();
    
    curvature_min_radius_ = this->get_parameter("curvature_min_radius").as_double();
    max_lateral_deviation_ = this->get_parameter("max_lateral_deviation").as_double();
    use_curvature_regulation_ = this->get_parameter("use_curvature_regulation").as_bool();
    
    laser_topic_ = this->get_parameter("laser_topic").as_string();
    obstacle_distance_threshold_ = this->get_parameter("obstacle_distance_threshold").as_double();
    obstacle_angle_range_ = this->get_parameter("obstacle_angle_range").as_double();
    
    costmap_topic_ = this->get_parameter("costmap_topic").as_string();
    cost_scaling_dist_ = this->get_parameter("cost_scaling_dist").as_double();
    cost_scaling_gain_ = this->get_parameter("cost_scaling_gain").as_double();
    use_costmap_ = this->get_parameter("use_costmap").as_bool();

    // 注册参数变化回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PPNode::onParameterChange, this, _1));

    // 创建控制器
    spin_controller_ = std::make_unique<SpinController>(
        spin_kp_, max_angular_vel_, angle_tolerance_);
    line_tracker_ = std::make_unique<LineTracker>(
        lookahead_gain_, lookahead_min_, lookahead_max_,
        max_angular_vel_, position_tolerance_, min_linear_vel_,
        curvature_min_radius_, max_lateral_deviation_);
    line_tracker_->setUseCurvatureRegulation(use_curvature_regulation_);

    // 初始化 TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 位姿订阅（map 系 Odometry）
    if (pose_source_ == "odom_topic") {
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            pose_topic_, rclcpp::QoS(10),
            std::bind(&PPNode::poseCallback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Pose source: odom_topic=%s (expect frame_id=%s)",
                    pose_topic_.c_str(), global_frame_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Pose source: TF %s->%s",
                    global_frame_.c_str(), base_frame_.c_str());
    }

    // 订阅激光雷达
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PPNode::laserCallback, this, _1));
    
    // 订阅 Costmap
    if (use_costmap_) {
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, rclcpp::QoS(1).transient_local(),
            std::bind(&PPNode::costmapCallback, this, _1));
    }

    // 发布话题
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/pp/state", 10);
    progress_pub_ = this->create_publisher<std_msgs::msg::Float32>("/pp/progress", 10);
    obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pp/obstacle", 10);
    lateral_dev_pub_ = this->create_publisher<std_msgs::msg::Float32>("/pp/lateral_deviation", 10);
    curvature_pub_ = this->create_publisher<std_msgs::msg::Float32>("/pp/curvature", 10);

    // 创建服务
    load_path_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/pp/load_path", std::bind(&PPNode::handleLoadPath, this, _1, _2));
    start_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/pp/start", std::bind(&PPNode::handleStart, this, _1, _2));
    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/pp/stop", std::bind(&PPNode::handleStop, this, _1, _2));
    pause_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/pp/pause", std::bind(&PPNode::handlePause, this, _1, _2));
    resume_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/pp/resume", std::bind(&PPNode::handleResume, this, _1, _2));

    // 创建控制循环定时器
    auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&PPNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "PP Controller (RPP Enhanced) initialized");
    RCLCPP_INFO(this->get_logger(), "  Lookahead: [%.2f, %.2f] m, gain=%.2f", 
                lookahead_min_, lookahead_max_, lookahead_gain_);
    RCLCPP_INFO(this->get_logger(), "  Curvature regulation: %s, radius=%.2f", 
                use_curvature_regulation_ ? "ON" : "OFF", curvature_min_radius_);
    RCLCPP_INFO(this->get_logger(), "  Max lateral deviation: %.2f m", max_lateral_deviation_);
    RCLCPP_INFO(this->get_logger(), "  Costmap: %s", use_costmap_ ? costmap_topic_.c_str() : "OFF");

    // 如果指定了路径文件，尝试加载
    if (!path_file_.empty()) {
        try {
            path_task_ = PathParser::loadFromFile(path_file_);
            RCLCPP_INFO(this->get_logger(), "Loaded path with %zu segments", 
                        path_task_.paths.size());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load path file: %s", e.what());
        }
    }
}

void PPNode::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != global_frame_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Pose topic frame_id=%s, expected=%s (ignore)",
            msg->header.frame_id.c_str(), global_frame_.c_str());
        return;
    }

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    const double yaw = tf2::getYaw(q);

    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_yaw_ = yaw;
    current_vel_ = msg->twist.twist.linear.x;
    last_pose_time_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
    pose_valid_ = true;
}

bool PPNode::updatePose()
{
    if (pose_source_ == "odom_topic") {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!pose_valid_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for pose topic: %s", pose_topic_.c_str());
            return false;
        }
        const auto now = this->get_clock()->now();
        const double age = (now - last_pose_time_).seconds();
        if (pose_timeout_ > 0.0 && age > pose_timeout_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Pose timeout: %.3fs > %.3fs (topic=%s)",
                age, pose_timeout_, pose_topic_.c_str());
            return false;
        }
        return true;
    }

    // TF mode
    try {
        auto tf = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);

        current_x_ = tf.transform.translation.x;
        current_y_ = tf.transform.translation.y;

        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);
        current_yaw_ = tf2::getYaw(q);

        pose_valid_ = true;
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Failed to get %s->%s transform: %s",
            global_frame_.c_str(), base_frame_.c_str(), ex.what());
        pose_valid_ = false;
        return false;
    }
}

void PPNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    bool detected = false;
    
    const double angle_min = msg->angle_min;
    const double angle_increment = msg->angle_increment;
    const size_t num_readings = msg->ranges.size();
    
    for (size_t i = 0; i < num_readings; ++i) {
        double angle = angle_min + i * angle_increment;
        
        if (std::abs(angle) < obstacle_angle_range_) {
            double range = msg->ranges[i];
            
            if (range >= msg->range_min && range <= msg->range_max) {
                if (range < obstacle_distance_threshold_) {
                    detected = true;
                    break;
                }
            }
        }
    }
    
    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        obstacle_detected_ = detected;
    }
}

void PPNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_ = msg;
}

double PPNode::getCostFactor(double x, double y)
{
    if (!use_costmap_) {
        return 1.0;  // 无减速
    }
    
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_) {
        return 1.0;
    }
    
    // 转换世界坐标到 costmap 索引
    double origin_x = costmap_->info.origin.position.x;
    double origin_y = costmap_->info.origin.position.y;
    double resolution = costmap_->info.resolution;
    int width = costmap_->info.width;
    int height = costmap_->info.height;
    
    int mx = static_cast<int>((x - origin_x) / resolution);
    int my = static_cast<int>((y - origin_y) / resolution);
    
    if (mx < 0 || mx >= width || my < 0 || my >= height) {
        return 1.0;  // 超出范围
    }
    
    int index = my * width + mx;
    int8_t cost = costmap_->data[index];
    
    if (cost < 0) {
        return 1.0;  // 未知区域
    }
    
    // cost: 0-100, 100 = 障碍物
    // 当 cost > 某阈值时开始减速
    const int cost_threshold = 50;
    if (cost > cost_threshold) {
        // 减速因子: cost 越高减速越多
        double factor = 1.0 - (static_cast<double>(cost - cost_threshold) / 
                               (100.0 - cost_threshold)) * (1.0 - cost_scaling_gain_);
        return clamp(factor, cost_scaling_gain_, 1.0);
    }
    
    return 1.0;
}

rcl_interfaces::msg::SetParametersResult PPNode::onParameterChange(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        if (param.get_name() == "path_file") {
            std::string new_path = param.as_string();
            if (!new_path.empty()) {
                try {
                    path_task_ = PathParser::loadFromFile(new_path);
                    path_file_ = new_path;
                    current_segment_idx_ = 0;
                    state_ = ControllerState::IDLE;
                    lateral_deviation_exceeded_ = false;
                    RCLCPP_INFO(this->get_logger(), 
                        "Dynamically loaded path with %zu segments",
                        path_task_.paths.size());
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load path: %s", e.what());
                    result.successful = false;
                    result.reason = e.what();
                }
            }
        }
        else if (param.get_name() == "max_linear_vel") {
            max_linear_vel_ = param.as_double();
        }
        else if (param.get_name() == "use_curvature_regulation") {
            use_curvature_regulation_ = param.as_bool();
            if (line_tracker_) {
                line_tracker_->setUseCurvatureRegulation(use_curvature_regulation_);
            }
        }
    }
    
    return result;
}

void PPNode::controlLoop()
{
    // 发布状态
    publishState();
    publishProgress();
    publishObstacleStatus();
    publishDiagnostics();

    // 更新位姿（TF 或 map 系 Odometry topic）
    if (!updatePose()) {
        return;
    }

    // 如果暂停，不执行控制
    if (is_paused_) {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }
    
    // 检查障碍物
    bool obstacle_stop = false;
    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        obstacle_stop = obstacle_detected_;
    }
    
    if (obstacle_stop && state_ == ControllerState::LINE_TRACKING) {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Obstacle detected! Stopping.");
        return;
    }
    
    // 检查横向偏差超限
    if (lateral_deviation_exceeded_ && state_ == ControllerState::LINE_TRACKING) {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Lateral deviation exceeded (%.3f m > %.3f m)! Stopping.",
            current_lateral_dev_, max_lateral_deviation_);
        return;
    }

    geometry_msgs::msg::Twist cmd_vel;

    switch (state_) {
        case ControllerState::IDLE:
        case ControllerState::COMPLETED:
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            break;

        case ControllerState::SPINNING:
        {
            auto [vx, omega, done] = spin_controller_->compute(current_yaw_, target_yaw_);
            cmd_vel.linear.x = vx;
            cmd_vel.angular.z = omega;

            if (done) {
                state_ = ControllerState::LINE_TRACKING;
                RCLCPP_INFO(this->get_logger(), "Spinning complete, switching to LINE_TRACKING");
            }
            break;
        }

        case ControllerState::LINE_TRACKING:
        {
            if (current_segment_idx_ >= path_task_.paths.size()) {
                state_ = ControllerState::COMPLETED;
                RCLCPP_INFO(this->get_logger(), "All path segments completed!");
                break;
            }

            const auto& segment = path_task_.paths[current_segment_idx_];
            
            // 获取 costmap 代价因子
            double cost_factor = getCostFactor(current_x_, current_y_);
            
            // RPP 增强控制
            auto output = line_tracker_->compute(
                current_x_, current_y_, current_yaw_,
                current_vel_,
                segment,
                cost_factor);

            // 更新横向偏差状态
            current_lateral_dev_ = output.lateral_dev;
            if (output.lateral_warning) {
                lateral_deviation_exceeded_ = true;
                break;  // 停止控制
            }

            cmd_vel.linear.x = output.vx;
            cmd_vel.angular.z = output.omega;

            if (output.is_done) {
                current_segment_idx_++;
                RCLCPP_INFO(this->get_logger(), "Segment %zu completed", current_segment_idx_);

                if (current_segment_idx_ >= path_task_.paths.size()) {
                    state_ = ControllerState::COMPLETED;
                    RCLCPP_INFO(this->get_logger(), "All path segments completed!");
                } else {
                    const auto& next_segment = path_task_.paths[current_segment_idx_];
                    if (next_segment.start_spin == 1) {
                        target_yaw_ = PathParser::calculateTargetYaw(next_segment);
                        state_ = ControllerState::SPINNING;
                        RCLCPP_INFO(this->get_logger(), "Starting spin for segment %zu", 
                                    current_segment_idx_);
                    }
                }
            }
            break;
        }
    }

    // === 速度限幅 ===
    cmd_vel.linear.x = clamp(cmd_vel.linear.x, -max_linear_vel_, max_linear_vel_);
    cmd_vel.angular.z = clamp(cmd_vel.angular.z, -max_angular_vel_, max_angular_vel_);

    // 记录当前速度用于自适应前瞻
    current_vel_ = cmd_vel.linear.x;

    cmd_vel_pub_->publish(cmd_vel);
}

void PPNode::handleLoadPath(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (path_file_.empty()) {
        response->success = false;
        response->message = "No path file specified. Set 'path_file' parameter first.";
        return;
    }

    try {
        path_task_ = PathParser::loadFromFile(path_file_);
        current_segment_idx_ = 0;
        state_ = ControllerState::IDLE;
        lateral_deviation_exceeded_ = false;
        
        response->success = true;
        response->message = "Loaded " + std::to_string(path_task_.paths.size()) + " segments";
        RCLCPP_INFO(this->get_logger(), "Loaded path: %s", response->message.c_str());
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to load path: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

void PPNode::handleStart(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (path_task_.paths.empty()) {
        response->success = false;
        response->message = "No path loaded";
        return;
    }

    current_segment_idx_ = 0;
    is_paused_ = false;
    lateral_deviation_exceeded_ = false;

    const auto& first_segment = path_task_.paths[0];
    if (first_segment.start_spin == 1) {
        target_yaw_ = PathParser::calculateTargetYaw(first_segment);
        state_ = ControllerState::SPINNING;
    } else {
        state_ = ControllerState::LINE_TRACKING;
    }

    response->success = true;
    response->message = "Started path execution with " + std::to_string(path_task_.paths.size()) + " segments";
    RCLCPP_INFO(this->get_logger(), "Path execution started");
}

void PPNode::handleStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    state_ = ControllerState::IDLE;
    current_segment_idx_ = 0;
    is_paused_ = false;
    lateral_deviation_exceeded_ = false;

    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);

    response->success = true;
    response->message = "Stopped and reset";
    RCLCPP_INFO(this->get_logger(), "Path execution stopped");
}

void PPNode::handlePause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    is_paused_ = true;

    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);

    response->success = true;
    response->message = "Paused";
    RCLCPP_INFO(this->get_logger(), "Path execution paused");
}

void PPNode::handleResume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    is_paused_ = false;
    lateral_deviation_exceeded_ = false;  // 恢复时清除横向偏差标志

    response->success = true;
    response->message = "Resumed";
    RCLCPP_INFO(this->get_logger(), "Path execution resumed");
}

void PPNode::publishState()
{
    std_msgs::msg::String msg;
    msg.data = stateToString(state_);
    if (lateral_deviation_exceeded_) {
        msg.data += "_LATERAL_ERROR";
    }
    state_pub_->publish(msg);
}

void PPNode::publishProgress()
{
    if (path_task_.paths.empty()) {
        return;
    }

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(current_segment_idx_) / 
               static_cast<float>(path_task_.paths.size());
    progress_pub_->publish(msg);
}

void PPNode::publishObstacleStatus()
{
    std_msgs::msg::Bool msg;
    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        msg.data = obstacle_detected_;
    }
    obstacle_pub_->publish(msg);
}

void PPNode::publishDiagnostics()
{
    // 发布横向偏差
    std_msgs::msg::Float32 lateral_msg;
    lateral_msg.data = static_cast<float>(current_lateral_dev_);
    lateral_dev_pub_->publish(lateral_msg);
}

}  // namespace pp_controller

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pp_controller::PPNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
