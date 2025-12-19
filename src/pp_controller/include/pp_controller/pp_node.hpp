#ifndef PP_CONTROLLER__PP_NODE_HPP_
#define PP_CONTROLLER__PP_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pp_controller/state_machine.hpp"
#include "pp_controller/path_parser.hpp"
#include "pp_controller/spin_controller.hpp"
#include "pp_controller/line_tracker.hpp"

namespace pp_controller
{

/**
 * @brief PP Controller 主节点 (Regulated Pure Pursuit 增强版)
 * 
 * 增强功能：
 * - 使用 TF 获取 map 坐标系下的位姿
 * - 动态路径加载（通过参数）
 * - 速度限幅
 * - 障碍物检测停车
 * - RPP 增强: 最大前瞻距离、曲率调节、横向偏差检测、Costmap集成
 */
class PPNode : public rclcpp::Node
{
public:
    explicit PPNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // 回调函数
    void controlLoop();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    // TF 位姿更新
    bool updatePoseFromTF();
    
    // Costmap 代价查询
    double getCostFactor(double x, double y);
    
    // 参数回调
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter>& parameters);

    // 服务处理
    void handleLoadPath(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handleStart(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handleStop(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handlePause(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handleResume(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // 发布状态
    void publishState();
    void publishProgress();
    void publishObstacleStatus();
    void publishDiagnostics();

    // 核心参数
    double max_linear_vel_;
    double min_linear_vel_;
    double max_angular_vel_;
    double lookahead_gain_;
    double lookahead_min_;
    double lookahead_max_;
    double position_tolerance_;
    double angle_tolerance_;
    double spin_kp_;
    double control_rate_;
    std::string path_file_;
    
    // RPP 增强参数
    double curvature_min_radius_;      // 曲率调节半径
    double max_lateral_deviation_;      // 最大横向偏差
    bool use_curvature_regulation_;     // 是否使用曲率调节
    
    // 障碍物检测参数
    std::string laser_topic_;
    double obstacle_distance_threshold_;
    double obstacle_angle_range_;
    
    // Costmap 参数
    std::string costmap_topic_;
    double cost_scaling_dist_;          // 触发减速的障碍物距离
    double cost_scaling_gain_;          // 减速增益
    bool use_costmap_;                  // 是否使用 costmap

    // 状态
    ControllerState state_;
    PathTask path_task_;
    size_t current_segment_idx_;
    double target_yaw_;
    bool is_paused_;
    
    // 障碍物状态
    bool obstacle_detected_;
    std::mutex obstacle_mutex_;
    
    // 横向偏差状态
    bool lateral_deviation_exceeded_;
    double current_lateral_dev_;
    
    // Costmap 数据
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    std::mutex costmap_mutex_;

    // 当前位姿 (map 坐标系)
    double current_x_;
    double current_y_;
    double current_yaw_;
    double current_vel_;  // 当前速度 (用于自适应前瞻)
    bool pose_valid_;

    // 控制器
    std::unique_ptr<SpinController> spin_controller_;
    std::unique_ptr<LineTracker> line_tracker_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lateral_dev_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr curvature_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_path_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // 参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace pp_controller

#endif  // PP_CONTROLLER__PP_NODE_HPP_
