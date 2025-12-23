#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rviz_common/display_context.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>

#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QFileDialog>
#include <QString>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <nav2_msgs/action/follow_path.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "waypoint_editor/io/waypoint_csv.hpp"
#include "waypoint_editor/io/waypoint_yaml.hpp"
#include "waypoint_editor/io/waypoint_json.hpp"
#include "waypoint_editor/rviz/waypoint_editor_tool.hpp"

using namespace std::placeholders;

namespace waypoint_editor
{

WaypointEditorTool::WaypointEditorTool() : rviz_default_plugins::tools::PoseTool() {}
WaypointEditorTool::~WaypointEditorTool() {}

void WaypointEditorTool::onInitialize()
{
    PoseTool::onInitialize();

    setName("Add Waypoint");

    nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    
    // Enable simulation time to match the navigation stack
    if (!nh_->has_parameter("use_sim_time")) {
        nh_->declare_parameter("use_sim_time", true);
    }
    nh_->set_parameter(rclcpp::Parameter("use_sim_time", true));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto_pose_topic_ = nh_->declare_parameter<std::string>("auto_pose_topic", "amcl_pose");
    auto_pose_type_  = nh_->declare_parameter<std::string>("auto_pose_type", "geometry_msgs/msg/PoseWithCovarianceStamped");
    auto_min_distance_m_ = nh_->declare_parameter<double>("auto_min_distance", 1.0);

    path_topic_ = nh_->declare_parameter<std::string>("path_topic", path_topic_);
    path_interpolation_step_m_ = nh_->declare_parameter<double>("path_interpolation_step_m", path_interpolation_step_m_);
    path_use_waypoint_orientation_ =
      nh_->declare_parameter<bool>("path_use_waypoint_orientation", path_use_waypoint_orientation_);
    publish_path_on_change_ = nh_->declare_parameter<bool>("publish_path_on_change", publish_path_on_change_);

    follow_path_action_name_ =
      nh_->declare_parameter<std::string>("follow_path_action_name", follow_path_action_name_);
    follow_path_controller_id_ =
      nh_->declare_parameter<std::string>("follow_path_controller_id", follow_path_controller_id_);
    follow_path_goal_checker_id_ =
      nh_->declare_parameter<std::string>("follow_path_goal_checker_id", follow_path_goal_checker_id_);
    param_cb_handle_ = nh_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &p : params) {
                if (p.get_name() == "auto_pose_topic") {
                    auto_pose_topic_ = p.as_string();
                } else if (p.get_name() == "auto_pose_type") {
                    const auto type = p.as_string();
                    if (type == "geometry_msgs/msg/PoseWithCovarianceStamped" || type == "PoseWithCovarianceStamped" ||
                        type == "geometry_msgs/msg/PoseStamped" || type == "PoseStamped") {
                        auto_pose_type_ = type;
                    } else {
                        result.successful = false;
                        result.reason = "Unsupported auto_pose_type";
                    }
                }
            }
            if (result.successful) {
                refreshAutoPoseSubscription();
            }
            return result;
        }
    );

    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "interactive_marker_server",
        nh_,
        rclcpp::SystemDefaultsQoS(),
        rclcpp::SystemDefaultsQoS()
    );
    save_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "save_waypoints",
        std::bind(&WaypointEditorTool::handleSaveWaypoints, this, _1, _2)
    );
    load_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "load_waypoints",
        std::bind(&WaypointEditorTool::handleLoadWaypoints, this, _1, _2)
    );
    undo_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "undo_waypoints",
        std::bind(&WaypointEditorTool::handleUndoWaypoints, this, _1, _2)
    );
    redo_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "redo_waypoints",
        std::bind(&WaypointEditorTool::handleRedoWaypoints, this, _1, _2)
    );
    clear_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "clear_waypoints",
        std::bind(&WaypointEditorTool::handleClearWaypoints, this, std::placeholders::_1, std::placeholders::_2));
    
    execute_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "execute_waypoints",
        std::bind(&WaypointEditorTool::handleExecuteWaypoints, this, std::placeholders::_1, std::placeholders::_2));

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(nh_, "follow_waypoints");

    publish_path_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "publish_path",
        std::bind(&WaypointEditorTool::handlePublishPath, this, std::placeholders::_1, std::placeholders::_2));

    execute_path_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "execute_path",
        std::bind(&WaypointEditorTool::handleExecutePath, this, std::placeholders::_1, std::placeholders::_2));

    follow_path_client_ =
      rclcpp_action::create_client<nav2_msgs::action::FollowPath>(nh_, follow_path_action_name_);

    auto_start_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "start_auto_waypoints",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            auto_enabled_ = true;
            res->success = true;
            res->message = "Auto waypoint capture started\n(topic: " + auto_pose_topic_ + ")";
            RCLCPP_INFO(nh_->get_logger(), "Auto waypoint capture started");
        }
    );
    auto_stop_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "stop_auto_waypoints",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            auto_enabled_ = false;
            res->success = true;
            res->message = "Auto waypoint capture stopped";
            RCLCPP_INFO(nh_->get_logger(), "Auto waypoint capture stopped");
        }
    );

    line_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("waypoint_line", 10);
    path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>(
        path_topic_, rclcpp::QoS(1).transient_local().reliable());
    total_wp_dist_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("total_wp_dist", 10);
    last_wp_dist_pub_  = nh_->create_publisher<std_msgs::msg::Float64>("last_wp_dist", 10);
    auto_distance_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
        "auto_waypoint_min_distance", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            auto_min_distance_m_ = std::max(0.0, msg->data);
            RCLCPP_INFO(nh_->get_logger(), "Auto waypoint min distance set to %.3f m", auto_min_distance_m_);
        }
    );
    
    // Subscribe to path interpolation density from Panel ComboBox
    auto path_density_sub = nh_->create_subscription<std_msgs::msg::Float64>(
        "path_interpolation_density", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            path_interpolation_step_m_ = std::max(0.01, msg->data);
            RCLCPP_INFO(nh_->get_logger(), "Path interpolation density set to %.2f m", path_interpolation_step_m_);
        }
    );
    
    refreshAutoPoseSubscription();

    waypoint_sequence_.clear();
    pose_dirty_ = false;
    updateLastDistanceFromWaypoint(0);
    publishRangeMetrics();
}

void WaypointEditorTool::onPoseSet(double x, double y, double theta)
{
    Waypoint wp;
    wp.pose.header.frame_id = "map";
    wp.pose.header.stamp = nh_->now();
    wp.pose.pose.position.x = x;
    wp.pose.pose.position.y = y;
    wp.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    wp.pose.pose.orientation.x = 0.0;
    wp.pose.pose.orientation.y = 0.0;
    wp.pose.pose.orientation.z = q.z();
    wp.pose.pose.orientation.w = q.w();

    wp.function_command.clear();

    const int new_id = appendWaypointAndRefresh(std::move(wp));
    RCLCPP_INFO(nh_->get_logger(), "Added waypoint %d", new_id);

    deactivate();
}

void WaypointEditorTool::updateWaypointMarker()
{
    server_->clear();
    for (size_t i = 0; i < waypoint_sequence_.size(); ++i) {
        auto int_marker = createWaypointMarker(static_cast<int>(i));
        server_->insert(int_marker, std::bind(&WaypointEditorTool::processFeedback, this, _1));
    }

    server_->applyChanges();
}

int WaypointEditorTool::appendWaypointAndRefresh(Waypoint wp)
{
    if (wp.pose.header.frame_id.empty()) {
        wp.pose.header.frame_id = "map";
    }
    if (wp.pose.header.stamp.sec == 0 && wp.pose.header.stamp.nanosec == 0) {
        wp.pose.header.stamp = nh_->now();
    }

    const int new_id = waypoint_sequence_.appendWaypoint(std::move(wp));
    auto int_marker = createWaypointMarker(new_id);
    server_->insert(int_marker, std::bind(&WaypointEditorTool::processFeedback, this, _1));
    server_->applyChanges();
    commitWaypointChanges(new_id);
    return new_id;
}

void WaypointEditorTool::handleAutoPose(const geometry_msgs::msg::PoseStamped &pose_in)
{
    if (!auto_enabled_) {
        return;
    }

    geometry_msgs::msg::PoseStamped pose_map;
    if (!transformToMapFrame(pose_in, pose_map)) {
        return;
    }

    const auto &waypoints = waypoint_sequence_.waypoints();
    if (!waypoints.empty()) {
        const auto &last_pose = waypoints.back().pose.pose.position;
        const double dist = std::hypot(
            pose_map.pose.position.x - last_pose.x,
            pose_map.pose.position.y - last_pose.y
        );
        if (dist < auto_min_distance_m_) {
            return;
        }
    }

    Waypoint wp;
    wp.pose = pose_map;
    wp.function_command.clear();
    appendWaypointAndRefresh(std::move(wp));
    RCLCPP_INFO(nh_->get_logger(), "Auto-added waypoint at (%.2f, %.2f)", pose_map.pose.position.x, pose_map.pose.position.y);
}

bool WaypointEditorTool::transformToMapFrame(const geometry_msgs::msg::PoseStamped &input, geometry_msgs::msg::PoseStamped &output) const
{
    if (!tf_buffer_) {
        return false;
    }

    output = input;
    if (output.header.frame_id.empty()) {
        output.header.frame_id = "map";
    }
    if (output.header.frame_id == "map") {
        return true;
    }

    try {
        const auto tf = tf_buffer_->lookupTransform("map", output.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(input, output, tf);
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
            nh_->get_logger(),
            *nh_->get_clock(),
            5000,
            "Failed to transform from %s to map: %s",
            output.header.frame_id.c_str(),
            ex.what());
        return false;
    }
}

void WaypointEditorTool::refreshAutoPoseSubscription()
{
    auto_pose_sub_.reset();
    const std::string type = auto_pose_type_;

    if (type == "geometry_msgs/msg/PoseWithCovarianceStamped" || type == "PoseWithCovarianceStamped") {
        auto_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            auto_pose_topic_, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                geometry_msgs::msg::PoseStamped pose_in;
                pose_in.header = msg->header;
                pose_in.pose = msg->pose.pose;
                handleAutoPose(pose_in);
            }
        );
        RCLCPP_INFO(nh_->get_logger(), "Auto pose subscription: %s (PoseWithCovarianceStamped)", auto_pose_topic_.c_str());
    } else if (type == "geometry_msgs/msg/PoseStamped" || type == "PoseStamped") {
        auto_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
            auto_pose_topic_, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                handleAutoPose(*msg);
            }
        );
        RCLCPP_INFO(nh_->get_logger(), "Auto pose subscription: %s (PoseStamped)", auto_pose_topic_.c_str());
    } else {
        RCLCPP_WARN(nh_->get_logger(), "Unsupported auto_pose_type: %s", type.c_str());
    }
}

visualization_msgs::msg::InteractiveMarker WaypointEditorTool::createWaypointMarker(const int id)
{
    const auto & wp = waypoint_sequence_.at(id);

    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = wp.pose.header.frame_id;
    int_marker.name = std::to_string(id);
    int_marker.description = waypoint_sequence_.at(id).function_command;
    int_marker.scale = 1.0;
    int_marker.pose.position = wp.pose.pose.position;
    int_marker.pose.orientation.x = 0.0;
    int_marker.pose.orientation.y = 0.0;
    int_marker.pose.orientation.z = wp.pose.pose.orientation.z;
    int_marker.pose.orientation.w = wp.pose.pose.orientation.w;

    // Position control (sphere)
    visualization_msgs::msg::InteractiveMarkerControl pos_control;
    pos_control.name = "move_position";
    pos_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    pos_control.always_visible = true;
    pos_control.orientation.w = 0.7071;
    pos_control.orientation.x = 0.0;
    pos_control.orientation.y = 0.7071;
    pos_control.orientation.z = 0.0;
    {
        visualization_msgs::msg::Marker sphere;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.scale.x = 0.4;
        sphere.scale.y = 0.4;
        sphere.scale.z = 0.4;
        sphere.color.r = 0.0;
        sphere.color.g = 1.0;
        sphere.color.b = 0.0;
        sphere.color.a = 1.0;
        pos_control.markers.push_back(sphere);
    }
    int_marker.controls.push_back(pos_control);

    visualization_msgs::msg::InteractiveMarkerControl rot_control_default;
    rot_control_default.name = "rotate_yaw_default";
    rot_control_default.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    rot_control_default.orientation.w = 0.7071;
    rot_control_default.orientation.x = 0.0;
    rot_control_default.orientation.y = -0.7071;
    rot_control_default.orientation.z = 0.0;
    rot_control_default.always_visible = true;
    int_marker.controls.push_back(rot_control_default);

    visualization_msgs::msg::InteractiveMarkerControl rot_control_arrow;
    rot_control_arrow.name = "rotate_yaw_arrow";
    rot_control_arrow.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    rot_control_arrow.orientation.w = 0.7071;
    rot_control_arrow.orientation.x = 0.0;
    rot_control_arrow.orientation.y = -0.7071;
    rot_control_arrow.orientation.z = 0.0;
    {
        visualization_msgs::msg::Marker arrow;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.scale.x = 0.4;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;
        rot_control_arrow.markers.push_back(arrow);
    }
    rot_control_arrow.always_visible = true;
    int_marker.controls.push_back(rot_control_arrow);

    // Text control
    visualization_msgs::msg::InteractiveMarkerControl text_control;
    text_control.name = "display_text";
    text_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
    text_control.always_visible = true;
    {
        visualization_msgs::msg::Marker text;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.scale.z = 0.2;
        text.color.r = 0.0;
        text.color.g = 0.0;
        text.color.b = 0.0;
        text.color.a = 1.0;
        std::string id_text = "ID:" + std::to_string(id) + "\n" + waypoint_sequence_.at(id).function_command;
        text.text = id_text;
        text.pose.position.x = -0.3;
        text.pose.position.y = -0.3;
        text.pose.position.z = 0.3;
        text_control.markers.push_back(text);
    }
    int_marker.controls.push_back(text_control);

    visualization_msgs::msg::InteractiveMarkerControl menu_control;
    menu_control.name              = "menu";
    menu_control.always_visible    = true;
    menu_control.interaction_mode  = visualization_msgs::msg::InteractiveMarkerControl::MENU;
    int_marker.controls.push_back(menu_control);
    
    visualization_msgs::msg::MenuEntry delete_entry;
    delete_entry.id = 1;
    delete_entry.parent_id = 0;
    delete_entry.title = "Delete Waypoint";
    int_marker.menu_entries.push_back(delete_entry);

    visualization_msgs::msg::MenuEntry change_id_entry;
    change_id_entry.id = 2;
    change_id_entry.parent_id = 0;
    change_id_entry.title = "Change Waypoint ID";
    int_marker.menu_entries.push_back(change_id_entry);
    
    visualization_msgs::msg::MenuEntry add_function_command_entry;
    add_function_command_entry.id = 3;
    add_function_command_entry.parent_id = 0;
    add_function_command_entry.title = "Edit Function Command";
    int_marker.menu_entries.push_back(add_function_command_entry);

    return int_marker;
}

void WaypointEditorTool::processFeedback(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> &fb)
{
    const int id = std::stoi(fb->marker_name);
    if (!isValidWaypointId(id)) {
        return;
    }

    switch (fb->event_type)
    {
        case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
        {
            geometry_msgs::msg::Pose new_pose = fb->pose;
            new_pose.orientation.x = 0.0;
            new_pose.orientation.y = 0.0;
            new_pose.orientation.z = fb->pose.orientation.z;
            new_pose.orientation.w = fb->pose.orientation.w;
            waypoint_sequence_.updatePose(id, new_pose);
            pose_dirty_ = true;

            server_->setPose(fb->marker_name, fb->pose);
            server_->applyChanges();
            updateLastDistanceFromWaypoint(id);
            publishRangeMetrics();
            break;
        }

        case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
        {
            if (pose_dirty_) {
                commitWaypointChanges(id);
            }
            break;
        }

        case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
        {
            processMenuControl(fb);
            break;
        }

        default:
            break;
    }
}

void WaypointEditorTool::processMenuControl(const std::shared_ptr<const visualization_msgs::msg::InteractiveMarkerFeedback> & fb)
{
    if (fb->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) { return; }

    const int id = std::stoi(fb->marker_name);
    if (!isValidWaypointId(id)) { return; }

    switch (fb->menu_entry_id) {
      
        // Delete Waypoint
        case 1:
            waypoint_sequence_.eraseWaypoint(static_cast<std::size_t>(id));
            updateWaypointMarker();
            commitWaypointChanges(id);
            RCLCPP_INFO(nh_->get_logger(), "Deleted waypoint %d", id);
            break;

        // Change Waypoint ID
        case 2:
        {
         bool ok = false;
            QString current = QString::fromStdString(std::to_string(id));
            QString text = QInputDialog::getText(
                nullptr,
                tr("Change Waypoint ID"),
                tr("Enter New Waypoint ID: %1").arg(id),
                QLineEdit::Normal,
                current,
                &ok
            );

            if (ok) {
                int insert_id = text.toInt();
                if (0 <= insert_id && insert_id < static_cast<int>(waypoint_sequence_.size())) {
                    Waypoint waypoint = waypoint_sequence_.at(static_cast<std::size_t>(id));
                    waypoint_sequence_.eraseWaypoint(static_cast<std::size_t>(id));
                    waypoint_sequence_.insertWaypoint(static_cast<std::size_t>(insert_id), std::move(waypoint));
                    updateWaypointMarker();
                    commitWaypointChanges(insert_id);
                    RCLCPP_INFO(nh_->get_logger(), "Changed waypoint id %d to %d", id, insert_id);
                } else {
                    const int max_index = std::max(0, static_cast<int>(waypoint_sequence_.size()) - 1);
                    QMessageBox::warning(
                        nullptr,
                        tr("Invalid Waypoint ID"),
                        tr("Waypoint ID %1 is out of range.\n"
                        "Please enter a value between %2 and %3.")
                        .arg(insert_id)
                        .arg(0)
                        .arg(max_index)
                    );
                }
            }
        }
            break;

        // Add / Edit Function Command
        case 3:
        {
            bool ok = false;
            QString current = QString::fromStdString(waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command);
            QString text = QInputDialog::getText(
                nullptr,
                tr("Edit Function Command"),
                tr("Enter command for waypoint ID: %1").arg(id),
                QLineEdit::Normal,
                current,
                &ok
            );

            if (ok) {
                waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command = text.toStdString();
                updateWaypointMarker();
                commitWaypointChanges(id);
                RCLCPP_INFO(nh_->get_logger(), "Updated command of waypoint %d to '%s'", id, waypoint_sequence_.at(static_cast<std::size_t>(id)).function_command.c_str());
            }
        }
            break;

      default:
            break;
    }
}

double WaypointEditorTool::computeSegmentDistance(std::size_t first, std::size_t second) const
{
    const auto &waypoints = waypoint_sequence_.waypoints();
    if (first >= waypoints.size() || second >= waypoints.size()) {
        return 0.0;
    }
    const auto &p0 = waypoints[first].pose.pose.position;
    const auto &p1 = waypoints[second].pose.pose.position;
    return std::hypot(p1.x - p0.x, p1.y - p0.y);
}

void WaypointEditorTool::updateLastDistanceFromWaypoint(int waypoint_index)
{
    const auto size = waypoint_sequence_.size();
    if (size < 2) {
        last_displayed_distance_ = 0.0;
        return;
    }

    const int max_index = static_cast<int>(size) - 1;
    const int clamped = std::max(0, std::min(waypoint_index, max_index));
    const std::size_t idx = static_cast<std::size_t>(clamped);

    if (idx > 0) {
        last_displayed_distance_ = computeSegmentDistance(idx - 1, idx);
    } else if (idx + 1 < size) {
        last_displayed_distance_ = computeSegmentDistance(idx, idx + 1);
    } else {
        last_displayed_distance_ = computeSegmentDistance(size - 2, size - 1);
    }
}

void WaypointEditorTool::publishLineMarker()
{
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = nh_->now();
    line.ns = "waypoint_lines";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action  = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.025f;
    line.color.r = 0.0f;
    line.color.g  = 1.0f;
    line.color.b  = 0.0f;
    line.color.a  = 1.0f;

    const auto &waypoints = waypoint_sequence_.waypoints();
    for (size_t i = 1; i < waypoints.size(); ++i) {
        geometry_msgs::msg::Point p0 = waypoints[i-1].pose.pose.position;
        geometry_msgs::msg::Point p1 = waypoints[i].pose.pose.position;
        line.points.push_back(p0);
        line.points.push_back(p1);
    }
    line_pub_->publish(line);
}

void WaypointEditorTool::publishTotalWpsDist()
{
    std_msgs::msg::Float64 msg;
    msg.data = waypoint_sequence_.totalDistance();
    total_wp_dist_pub_->publish(msg);
}

void WaypointEditorTool::publishLastWpsDist()
{
    std_msgs::msg::Float64 msg;
    msg.data = last_displayed_distance_;
    last_wp_dist_pub_->publish(msg);
}

void WaypointEditorTool::publishRangeMetrics()
{
    publishLineMarker();
    publishPath();
    publishTotalWpsDist();
    publishLastWpsDist();
}

bool WaypointEditorTool::buildPath(nav_msgs::msg::Path &path_msg, std::string &error) const
{
    path_msg = nav_msgs::msg::Path();

    const auto &waypoints = waypoint_sequence_.waypoints();
    if (waypoints.size() < 2) {
        error = "Need at least 2 waypoints to build a path";
        return false;
    }

    std::vector<geometry_msgs::msg::PoseStamped> wps_map;
    wps_map.reserve(waypoints.size());
    for (const auto &wp : waypoints) {
        geometry_msgs::msg::PoseStamped pose_map;
        if (!transformToMapFrame(wp.pose, pose_map)) {
            error = "Failed to transform waypoint to map frame (frame_id=" + wp.pose.header.frame_id + ")";
            return false;
        }
        pose_map.header.stamp = nh_->now();
        pose_map.header.frame_id = "map";
        wps_map.push_back(std::move(pose_map));
    }

    path_msg.header.stamp = nh_->now();
    path_msg.header.frame_id = "map";

    const double step = std::max(0.0, path_interpolation_step_m_);

    auto setYaw = [](geometry_msgs::msg::Pose &pose, double yaw) {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
    };

    auto pushPose = [&](const geometry_msgs::msg::PoseStamped &p) {
        geometry_msgs::msg::PoseStamped out = p;
        out.header = path_msg.header;
        path_msg.poses.push_back(std::move(out));
    };

    pushPose(wps_map.front());

    for (size_t i = 0; i + 1 < wps_map.size(); ++i) {
        const auto &a = wps_map[i];
        const auto &b = wps_map[i + 1];

        const double ax = a.pose.position.x;
        const double ay = a.pose.position.y;
        const double bx = b.pose.position.x;
        const double by = b.pose.position.y;
        const double dx = bx - ax;
        const double dy = by - ay;
        const double dist = std::hypot(dx, dy);

        if (dist < 1e-9) {
            continue;
        }

        const double yaw = std::atan2(dy, dx);

        // No interpolation: just add the end waypoint.
        if (step <= 1e-9 || dist <= step) {
            geometry_msgs::msg::PoseStamped end_pose = b;
            if (!path_use_waypoint_orientation_) {
                setYaw(end_pose.pose, yaw);
            }
            pushPose(end_pose);
            continue;
        }

        // Interpolate poses along the segment.
        const int num_steps = static_cast<int>(std::floor(dist / step));
        for (int s = 1; s <= num_steps; ++s) {
            const double t = std::min(1.0, (s * step) / dist);

            geometry_msgs::msg::PoseStamped p = a;
            p.pose.position.x = ax + t * dx;
            p.pose.position.y = ay + t * dy;
            p.pose.position.z = 0.0;

            if (t >= 1.0 - 1e-9 && path_use_waypoint_orientation_) {
                p.pose.orientation = b.pose.orientation;
            } else {
                setYaw(p.pose, yaw);
            }

            // Avoid adding a point that duplicates the path tail.
            const auto &tail = path_msg.poses.back().pose.position;
            const double tail_dist = std::hypot(p.pose.position.x - tail.x, p.pose.position.y - tail.y);
            if (tail_dist > 1e-6) {
                pushPose(p);
            }
        }

        // Ensure end point exists.
        const auto &tail = path_msg.poses.back().pose.position;
        const double tail_to_end = std::hypot(tail.x - bx, tail.y - by);
        if (tail_to_end > 1e-6) {
            geometry_msgs::msg::PoseStamped end_pose = b;
            if (!path_use_waypoint_orientation_) {
                setYaw(end_pose.pose, yaw);
            }
            pushPose(end_pose);
        }
    }

    if (path_msg.poses.size() < 2) {
        error = "Path has less than 2 poses after processing";
        return false;
    }

    error.clear();
    return true;
}

void WaypointEditorTool::publishPath(bool force)
{
    if (!path_pub_) {
        return;
    }
    if (!force && !publish_path_on_change_) {
        return;
    }

    nav_msgs::msg::Path path_msg;
    std::string error;
    if (!buildPath(path_msg, error)) {
        return;
    }

    path_pub_->publish(path_msg);
}

void WaypointEditorTool::handlePublishPath(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    nav_msgs::msg::Path path_msg;
    std::string error;
    if (!buildPath(path_msg, error)) {
        res->success = false;
        res->message = error;
        return;
    }

    if (!path_pub_) {
        res->success = false;
        res->message = "Path publisher not initialized";
        return;
    }

    path_pub_->publish(path_msg);
    res->success = true;
    res->message = "Published path with " + std::to_string(path_msg.poses.size()) +
      " poses on " + std::string(path_pub_->get_topic_name());
}

void WaypointEditorTool::commitWaypointChanges(int waypoint_index, bool snapshot_history)
{
    if (snapshot_history) {
        waypoint_sequence_.snapshotHistory();
    }
    pose_dirty_ = false;
    updateLastDistanceFromWaypoint(waypoint_index);
    publishRangeMetrics();
}

bool WaypointEditorTool::isValidWaypointId(int id) const
{
    return id >= 0 && id < static_cast<int>(waypoint_sequence_.size());
}

bool WaypointEditorTool::requestFilePathForSaving(std::string &path, bool &save_as_yaml)
{
    QString selected_filter;
    QString qpath = QFileDialog::getSaveFileName(
        nullptr,
        tr("Save Waypoints As"),
        "",
        tr("JSON Path (*.txt *.json);;CSV Files (*.csv);;YAML Files (*.yaml)"),
        &selected_filter
    );

    if (qpath.isEmpty()) {
        return false;
    }

    auto lower = qpath.toLower();
    if (lower.endsWith(".yaml")) {
        save_as_yaml = true;
    } else if (lower.endsWith(".csv")) {
        save_as_yaml = false;
    } else if (lower.endsWith(".txt") || lower.endsWith(".json")) {
        // JSON 格式会在 handleSaveWaypoints 中单独处理
        save_as_yaml = false;
    } else {
        // Fallback to selected filter when no extension given.
        if (selected_filter.contains("JSON", Qt::CaseInsensitive)) {
            qpath += ".txt";
            save_as_yaml = false;
        } else if (selected_filter.contains("yaml", Qt::CaseInsensitive)) {
            qpath += ".yaml";
            save_as_yaml = true;
        } else {
            qpath += ".csv";
            save_as_yaml = false;
        }
    }

    path = qpath.toStdString();
    return true;
}

bool WaypointEditorTool::requestFilePathForLoading(std::string &path, bool &load_yaml)
{
    QString selected_filter;
    QString qpath = QFileDialog::getOpenFileName(
        nullptr,
        tr("Open Waypoints"),
        "",
        tr("All Supported (*.csv *.yaml *.json *.txt);;CSV Files (*.csv);;YAML Files (*.yaml);;JSON Path (*.json *.txt)"),
        &selected_filter
    );
    if (qpath.isEmpty()) {
        return false;
    }

    auto lower = qpath.toLower();
    if (lower.endsWith(".yaml")) {
        load_yaml = true;
    } else if (lower.endsWith(".csv")) {
        load_yaml = false;
    } else {
        load_yaml = selected_filter.contains("yaml", Qt::CaseInsensitive);
    }

    path = qpath.toStdString();
    return true;
}

void WaypointEditorTool::handleSaveWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    std::string path;
    bool save_as_yaml = false;
    if (!requestFilePathForSaving(path, save_as_yaml)) {
        res->success = false;
        res->message = "Save canceled by user";
        return;
    }

    std::string error;
    bool ok = false;
    
    // 检查是否为 JSON/TXT 格式（用于 PP Controller）
    auto lower_path = path;
    std::transform(lower_path.begin(), lower_path.end(), lower_path.begin(), ::tolower);
    
    // C++17 兼容的后缀检查
    auto endsWith = [](const std::string& str, const std::string& suffix) {
        if (suffix.size() > str.size()) return false;
        return str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
    };
    
    if (endsWith(lower_path, ".txt") || endsWith(lower_path, ".json")) {
        // 保存为 JSON 路径格式（PP Controller 格式）
        ok = io::WaypointJson::Save(waypoint_sequence_.waypoints(), path, error);
    } else if (save_as_yaml) {
        ok = io::WaypointYaml::Save(waypoint_sequence_.waypoints(), path, error);
    } else {
        ok = io::WaypointCsv::Save(waypoint_sequence_.waypoints(), path, error);
    }

    if (!ok) {
        QMessageBox::warning(nullptr, tr("Error"), tr("Cannot save file:\n%1\n%2").arg(QString::fromStdString(path)).arg(QString::fromStdString(error)));
        res->success = false;
        res->message = error;
        return;
    }

    // ===== 工业级路径: 自动保存插值后的路径到 wpfile 目录 =====
    std::string auto_save_msg;
    try {
        // 获取 path_interpolation_step_m_ (从 ROS 参数或使用默认值)
        double density = path_interpolation_step_m_;  // 默认 0.1m
        
        // 插值生成密集点
        auto interpolated = interpolateWaypoints(waypoint_sequence_.waypoints(), density);
        
        // 生成自动保存路径
        std::string auto_path = generateWpfilePath();
        
        // 保存插值后的路径为 JSON 格式
        std::string auto_error;
        if (io::WaypointJson::Save(interpolated, auto_path, auto_error)) {
            auto_save_msg = "\nAuto-saved " + std::to_string(interpolated.size()) + 
                           " interpolated points to:\n" + auto_path;
            RCLCPP_INFO(nh_->get_logger(), "Auto-saved interpolated path to: %s", auto_path.c_str());
        } else {
            RCLCPP_WARN(nh_->get_logger(), "Failed to auto-save: %s", auto_error.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(nh_->get_logger(), "Auto-save failed: %s", e.what());
    }

    res->success = true;
    res->message = "Saved " + std::to_string(waypoint_sequence_.size()) + " waypoints to " + path + auto_save_msg;
}

void WaypointEditorTool::handleLoadWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    std::string path;
    bool load_yaml = false;
    if (!requestFilePathForLoading(path, load_yaml)) {
        res->success = false;
        res->message = "Load canceled by user";
        return;
    }

    std::vector<Waypoint> loaded;
    std::string error;
    bool ok = false;
    
    // Check extension
    auto lower_path = path;
    std::transform(lower_path.begin(), lower_path.end(), lower_path.begin(), ::tolower);
    auto endsWith = [](const std::string& str, const std::string& suffix) {
        if (suffix.size() > str.size()) return false;
        return str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
    };

    if (endsWith(lower_path, ".json") || endsWith(lower_path, ".txt")) {
        ok = io::WaypointJson::Load(path, loaded, error);
    } else if (load_yaml) {
        ok = io::WaypointYaml::Load(path, loaded, error);
    } else {
        ok = io::WaypointCsv::Load(path, loaded, error);
    }

    if (!ok) {
        QMessageBox::warning(nullptr, tr("Error"), tr("Cannot open file:\n%1\n%2").arg(QString::fromStdString(path)).arg(QString::fromStdString(error)));
        res->success = false;
        res->message = error;
        return;
    }

    for (auto &wp : loaded) {
        if (wp.pose.header.frame_id.empty()) {
            wp.pose.header.frame_id = "map";
        }
        wp.pose.header.stamp = nh_->now();
    }
    waypoint_sequence_.assign(std::move(loaded));
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1);

    res->success = true;
    res->message = "Loaded " + std::to_string(waypoint_sequence_.size()) + " waypoints from " + path;
}

void WaypointEditorTool::handleUndoWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!waypoint_sequence_.undo()) {
        res->success = false;
        res->message = "No more actions to undo";
        return;
    }
    pose_dirty_ = false;
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1, false);
    res->success = true;
    res->message = "Undid waypoint change";
}

void WaypointEditorTool::handleRedoWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!waypoint_sequence_.redo()) {
        res->success = false;
        res->message = "No more actions to redo";
        return;
    }
    pose_dirty_ = false;
    updateWaypointMarker();
    commitWaypointChanges(static_cast<int>(waypoint_sequence_.size()) - 1, false);
    res->success = true;
    res->message = "Redid waypoint change";
}

void WaypointEditorTool::handleClearWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    
    // Cancel any ongoing navigation first
    if (current_goal_handle_) {
        RCLCPP_INFO(nh_->get_logger(), "Canceling ongoing navigation...");
        nav_client_->async_cancel_goal(current_goal_handle_);
        current_goal_handle_.reset();
    }

    if (current_path_goal_handle_ && follow_path_client_) {
        RCLCPP_INFO(nh_->get_logger(), "Canceling ongoing path tracking...");
        follow_path_client_->async_cancel_goal(current_path_goal_handle_);
        current_path_goal_handle_.reset();
    }
    
    waypoint_sequence_.clear();
    server_->clear();
    server_->applyChanges();
    publishLineMarker();
    publishPath(true);
    res->success = true;
    res->message = "Waypoints cleared and navigation canceled";
}

void WaypointEditorTool::handleExecuteWaypoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;

  if (waypoint_sequence_.size() == 0) {
    res->success = false;
    res->message = "No waypoints to execute";
    RCLCPP_WARN(nh_->get_logger(), "Cannot execute: No waypoints defined.");
    return;
  }

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    res->success = false;
    res->message = "Action server 'follow_waypoints' not available";
    RCLCPP_ERROR(nh_->get_logger(), "Action server 'follow_waypoints' not available");
    return;
  }

  // Convert waypoints to goal
  auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
  std::vector<Waypoint> waypoints = waypoint_sequence_.waypoints();
  
  for (const auto &wp : waypoints) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = wp.pose.header.frame_id; // Use wp.pose.header.frame_id
    pose_msg.header.stamp = nh_->now();
    pose_msg.pose = wp.pose.pose; // Use wp.pose.pose
    goal_msg.poses.push_back(pose_msg);
  }

  // Send goal
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr & goal_handle) {
      current_goal_handle_ = goal_handle;
      if (!goal_handle) {
        RCLCPP_ERROR(nh_->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(nh_->get_logger(), "Goal accepted by server");
      }
    };

  send_goal_options.result_callback = 
    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult & result) {
      current_goal_handle_.reset();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(nh_->get_logger(), "Navigation succeeded");
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_INFO(nh_->get_logger(), "Navigation canceled");
      } else {
        RCLCPP_ERROR(nh_->get_logger(), "Navigation failed");
      }
    };

  nav_client_->async_send_goal(goal_msg, send_goal_options);

  res->success = true;
  res->message = "Navigation goal sent with " + std::to_string(goal_msg.poses.size()) + " waypoints";
  RCLCPP_INFO(nh_->get_logger(), "Sent %zu waypoints to Nav2", goal_msg.poses.size());
}

void WaypointEditorTool::handleExecutePath(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (!follow_path_client_) {
    res->success = false;
    res->message = "FollowPath action client not initialized";
    return;
  }

  nav_msgs::msg::Path path_msg;
  std::string error;
  if (!buildPath(path_msg, error)) {
    res->success = false;
    res->message = error;
    return;
  }

  if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(2))) {
    res->success = false;
    res->message = "Action server '" + follow_path_action_name_ + "' not available";
    RCLCPP_ERROR(nh_->get_logger(), "Action server '%s' not available", follow_path_action_name_.c_str());
    return;
  }

  auto goal_msg = nav2_msgs::action::FollowPath::Goal();
  goal_msg.path = path_msg;
  goal_msg.controller_id = follow_path_controller_id_;
  goal_msg.goal_checker_id = follow_path_goal_checker_id_;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr & goal_handle) {
      current_path_goal_handle_ = goal_handle;
      if (!goal_handle) {
        RCLCPP_ERROR(nh_->get_logger(), "FollowPath goal was rejected by server");
      } else {
        RCLCPP_INFO(nh_->get_logger(), "FollowPath goal accepted by server");
      }
    };

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult & result) {
      current_path_goal_handle_.reset();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(nh_->get_logger(), "Path tracking succeeded");
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_INFO(nh_->get_logger(), "Path tracking canceled");
      } else {
        RCLCPP_ERROR(nh_->get_logger(), "Path tracking failed");
      }
    };

  follow_path_client_->async_send_goal(goal_msg, send_goal_options);

  // Publish for visualization / other consumers
  if (path_pub_) {
    path_pub_->publish(path_msg);
  }

  res->success = true;
  res->message = "FollowPath goal sent with " + std::to_string(path_msg.poses.size()) + " poses";
}

void WaypointEditorTool::activate() {}
void WaypointEditorTool::deactivate()
{
    PoseTool::deactivate();
}

std::vector<Waypoint> WaypointEditorTool::interpolateWaypoints(
    const std::vector<Waypoint>& waypoints, double density_m) const
{
    if (waypoints.size() < 2 || density_m <= 0.0) {
        return waypoints;
    }

    std::vector<Waypoint> result;
    result.push_back(waypoints.front());

    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        const auto& prev = waypoints[i - 1];
        const auto& curr = waypoints[i];

        double dx = curr.pose.pose.position.x - prev.pose.pose.position.x;
        double dy = curr.pose.pose.position.y - prev.pose.pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist <= density_m) {
            result.push_back(curr);
            continue;
        }

        int num_segments = static_cast<int>(std::ceil(dist / density_m));
        double step_x = dx / num_segments;
        double step_y = dy / num_segments;

        // Interpolate intermediate points
        for (int j = 1; j < num_segments; ++j) {
            Waypoint interp = prev;
            interp.pose.pose.position.x = prev.pose.pose.position.x + step_x * j;
            interp.pose.pose.position.y = prev.pose.pose.position.y + step_y * j;
            interp.pose.header.stamp = nh_->now();
            result.push_back(interp);
        }
        result.push_back(curr);
    }

    RCLCPP_INFO(nh_->get_logger(), "Interpolated %zu waypoints to %zu points (density: %.2f m)",
                waypoints.size(), result.size(), density_m);
    return result;
}

std::string WaypointEditorTool::generateWpfilePath() const
{
    // Get package share directory for wpfile
    std::string pkg_share = ament_index_cpp::get_package_share_directory("waypoint_editor");
    // Replace share with src path for editable files
    std::string src_path = pkg_share;
    auto pos = src_path.find("/install/");
    if (pos != std::string::npos) {
        src_path = src_path.substr(0, pos) + "/src/robot_app/waypoint_editor/wpfile";
    } else {
        src_path = pkg_share + "/wpfile";
    }

    // Create directory if it doesn't exist
    std::filesystem::create_directories(src_path);

    // Generate timestamp filename
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    
    return src_path + "/" + ss.str() + ".json";
}

} // namespace waypoint_editor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_editor::WaypointEditorTool, rviz_common::Tool)
