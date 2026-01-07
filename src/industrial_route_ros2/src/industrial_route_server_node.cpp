#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "industrial_route_core/geojson_loader.hpp"
#include "industrial_route_core/map_matching.hpp"
#include "industrial_route_core/path_builder.hpp"
#include "industrial_route_core/route_planner.hpp"
#include "industrial_route_interfaces/action/execute_route.hpp"
#include "industrial_route_interfaces/msg/route_status.hpp"
#include "industrial_route_interfaces/srv/get_nearest_node.hpp"
#include "industrial_route_interfaces/srv/plan_route.hpp"
#include "industrial_route_interfaces/srv/update_edge.hpp"
#include "industrial_route_ros2/occupancy_grid_collision_checker.hpp"
#include "industrial_route_ros2/pure_pursuit_controller.hpp"
#include "industrial_route_ros2/rviz_markers.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace industrial_route_ros2 {
namespace {

double yawFromQuat(const geometry_msgs::msg::Quaternion& qmsg) {
  tf2::Quaternion q;
  tf2::fromMsg(qmsg, q);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion quatFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

industrial_route_core::Pose2D pose2DFromPoseStamped(const geometry_msgs::msg::PoseStamped& p) {
  industrial_route_core::Pose2D out;
  out.x = p.pose.position.x;
  out.y = p.pose.position.y;
  out.yaw = yawFromQuat(p.pose.orientation);
  return out;
}

geometry_msgs::msg::PoseStamped poseStampedFromPose2D(const industrial_route_core::Pose2D& p,
                                                      const std::string& frame_id,
                                                      const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = frame_id;
  out.header.stamp = stamp;
  out.pose.position.x = p.x;
  out.pose.position.y = p.y;
  out.pose.position.z = 0.0;
  out.pose.orientation = quatFromYaw(p.yaw);
  return out;
}

nav_msgs::msg::Path toPathMsg(const std::vector<industrial_route_core::Pose2D>& path,
                              const std::string& frame_id,
                              const rclcpp::Time& stamp) {
  nav_msgs::msg::Path out;
  out.header.frame_id = frame_id;
  out.header.stamp = stamp;
  out.poses.reserve(path.size());
  for (const auto& p : path) {
    out.poses.push_back(poseStampedFromPose2D(p, frame_id, stamp));
  }
  return out;
}

std::vector<industrial_route_core::Pose2D> fromPathMsg(const nav_msgs::msg::Path& path) {
  std::vector<industrial_route_core::Pose2D> out;
  out.reserve(path.poses.size());
  for (const auto& p : path.poses) {
    out.push_back(pose2DFromPoseStamped(p));
  }
  return out;
}

}  // namespace

class IndustrialRouteServer final : public rclcpp::Node {
public:
  using PlanRoute = industrial_route_interfaces::srv::PlanRoute;
  using GetNearestNode = industrial_route_interfaces::srv::GetNearestNode;
  using UpdateEdge = industrial_route_interfaces::srv::UpdateEdge;
  using ExecuteRoute = industrial_route_interfaces::action::ExecuteRoute;
  using GoalHandleExecuteRoute = rclcpp_action::ServerGoalHandle<ExecuteRoute>;

  IndustrialRouteServer()
      : rclcpp::Node("industrial_route_server"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    declareParams();
    loadPlannerAndControllerParams();

    graph_pub_ = create_publisher<industrial_route_interfaces::msg::RouteGraph>(
        "/industrial_route/graph", rclcpp::QoS(1).transient_local().reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/industrial_route/graph_markers", rclcpp::QoS(1).transient_local().reliable());
    status_pub_ = create_publisher<industrial_route_interfaces::msg::RouteStatus>(
        "/industrial_route/status", rclcpp::QoS(10));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/industrial_route/path", rclcpp::QoS(1).transient_local().reliable());

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));

    reload_srv_ = create_service<std_srvs::srv::Trigger>(
        "/industrial_route/reload_graph", std::bind(&IndustrialRouteServer::onReloadGraph, this, std::placeholders::_1, std::placeholders::_2));
    plan_srv_ = create_service<PlanRoute>(
        "/industrial_route/plan", std::bind(&IndustrialRouteServer::onPlanRoute, this, std::placeholders::_1, std::placeholders::_2));
    nearest_srv_ = create_service<GetNearestNode>(
        "/industrial_route/get_nearest_node", std::bind(&IndustrialRouteServer::onGetNearestNode, this, std::placeholders::_1, std::placeholders::_2));
    update_edge_srv_ = create_service<UpdateEdge>(
        "/industrial_route/update_edge", std::bind(&IndustrialRouteServer::onUpdateEdge, this, std::placeholders::_1, std::placeholders::_2));

    pause_srv_ = create_service<std_srvs::srv::SetBool>(
        "/industrial_route/pause", std::bind(&IndustrialRouteServer::onPause, this, std::placeholders::_1, std::placeholders::_2));
    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
        "/industrial_route/cancel", std::bind(&IndustrialRouteServer::onCancel, this, std::placeholders::_1, std::placeholders::_2));

    action_server_ = rclcpp_action::create_server<ExecuteRoute>(
        this,
        "/industrial_route/execute",
        std::bind(&IndustrialRouteServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&IndustrialRouteServer::handleCancel, this, std::placeholders::_1),
        std::bind(&IndustrialRouteServer::handleAccepted, this, std::placeholders::_1));

    if (use_map_collision_check_) {
      OccupancyGridCollisionChecker::Options cc_opts;
      cc_opts.occupied_threshold = get_parameter("collision.occupied_threshold").as_int();
      cc_opts.treat_unknown_as_occupied = get_parameter("collision.treat_unknown_as_occupied").as_bool();
      collision_checker_ = std::make_unique<OccupancyGridCollisionChecker>(this, map_topic_, cc_opts);
    }

    if (!graph_geojson_path_.empty()) {
      (void)reloadGraph(graph_geojson_path_);
    } else {
      RCLCPP_WARN(get_logger(), "graph_geojson_path is empty; server starts without a graph");
    }

    control_timer_ = create_wall_timer(50ms, std::bind(&IndustrialRouteServer::controlLoop, this));
  }

private:
  enum class ExecState { IDLE, EXECUTING, PAUSED };

  void declareParams() {
    declare_parameter<std::string>("graph_geojson_path", "");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("base_frame_id", "base_link");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<bool>("use_map_collision_check", false);

    declare_parameter<double>("planner.connect_max_dist", 6.0);
    declare_parameter<int>("planner.connect_k_nearest", 8);
    declare_parameter<double>("planner.on_graph_max_dist", 0.8);
    declare_parameter<double>("planner.densify_resolution", 0.2);
    declare_parameter<bool>("planner.use_edge_weight", true);
    declare_parameter<bool>("planner.use_max_speed_cost", false);

    declare_parameter<double>("controller.max_linear_speed", 0.5);
    declare_parameter<double>("controller.max_angular_speed", 1.2);
    declare_parameter<double>("controller.lookahead_min", 0.4);
    declare_parameter<double>("controller.lookahead_gain", 0.8);
    declare_parameter<double>("controller.goal_pos_tolerance", 0.15);
    declare_parameter<double>("controller.goal_yaw_tolerance", 0.25);
    declare_parameter<double>("controller.rotate_in_place_yaw_threshold", 0.6);
    declare_parameter<double>("controller.rotate_in_place_speed", 0.5);

    declare_parameter<int>("collision.occupied_threshold", 50);
    declare_parameter<bool>("collision.treat_unknown_as_occupied", true);

    graph_geojson_path_ = get_parameter("graph_geojson_path").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    use_map_collision_check_ = get_parameter("use_map_collision_check").as_bool();
  }

  void loadPlannerAndControllerParams() {
    planner_opts_.connect_max_dist = get_parameter("planner.connect_max_dist").as_double();
    planner_opts_.connect_k_nearest = static_cast<std::size_t>(get_parameter("planner.connect_k_nearest").as_int());
    planner_opts_.on_graph_max_dist = get_parameter("planner.on_graph_max_dist").as_double();
    planner_opts_.densify_resolution = get_parameter("planner.densify_resolution").as_double();
    planner_opts_.use_edge_weight = get_parameter("planner.use_edge_weight").as_bool();
    planner_opts_.use_max_speed_cost = get_parameter("planner.use_max_speed_cost").as_bool();

    PurePursuitController::Limits limits;
    limits.max_linear_speed = get_parameter("controller.max_linear_speed").as_double();
    limits.max_angular_speed = get_parameter("controller.max_angular_speed").as_double();

    PurePursuitController::Params params;
    params.lookahead_min = get_parameter("controller.lookahead_min").as_double();
    params.lookahead_gain = get_parameter("controller.lookahead_gain").as_double();
    params.goal_pos_tolerance = get_parameter("controller.goal_pos_tolerance").as_double();
    params.goal_yaw_tolerance = get_parameter("controller.goal_yaw_tolerance").as_double();
    params.rotate_in_place_yaw_threshold = get_parameter("controller.rotate_in_place_yaw_threshold").as_double();
    params.rotate_in_place_speed = get_parameter("controller.rotate_in_place_speed").as_double();

    controller_ = std::make_unique<PurePursuitController>(limits, params);
  }

  bool reloadGraph(const std::string& path) {
    industrial_route_core::Graph g;
    const auto res = industrial_route_core::GeoJsonLoader::loadFromFile(path, &g);
    if (!res.success) {
      RCLCPP_ERROR(get_logger(), "Failed to load graph: %s", res.error_msg.c_str());
      return false;
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      graph_ = std::move(g);
      route_planner_ = std::make_unique<industrial_route_core::RoutePlanner>(&graph_);
    }
    publishGraph();
    RCLCPP_INFO(get_logger(), "Loaded graph from %s (nodes=%zu edges=%zu)",
                path.c_str(), graph_.nodeIds().size(), graph_.edgeIds().size());
    return true;
  }

  void publishGraph() {
    auto msg = RvizMarkers::toMsg(graph_, frame_id_);
    auto markers = RvizMarkers::makeGraphMarkers(graph_, frame_id_);
    graph_pub_->publish(msg);
    marker_pub_->publish(markers);
  }

  std::optional<industrial_route_core::Pose2D> getRobotPose() {
    try {
      const auto tf = tf_buffer_.lookupTransform(frame_id_, base_frame_id_, tf2::TimePointZero);
      industrial_route_core::Pose2D p;
      p.x = tf.transform.translation.x;
      p.y = tf.transform.translation.y;
      p.yaw = yawFromQuat(tf.transform.rotation);
      return p;
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", e.what());
      return std::nullopt;
    }
  }

  void publishStatus(const std::string& state, float progress,
                     const std::string& current_edge_id,
                     const std::string& current_node_id,
                     const std::string& error_code = "", const std::string& error_msg = "") {
    industrial_route_interfaces::msg::RouteStatus st;
    st.header.frame_id = frame_id_;
    st.header.stamp = now();
    st.state = state;
    st.progress = progress;
    st.current_edge_id = current_edge_id;
    st.current_node_id = current_node_id;
    st.error_code = error_code;
    st.error_msg = error_msg;
    status_pub_->publish(st);
  }

  // Services
  void onReloadGraph(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    if (graph_geojson_path_.empty()) {
      resp->success = false;
      resp->message = "graph_geojson_path is empty";
      return;
    }
    resp->success = reloadGraph(graph_geojson_path_);
    resp->message = resp->success ? "ok" : "failed";
  }

  void onPlanRoute(const std::shared_ptr<PlanRoute::Request> req,
                   std::shared_ptr<PlanRoute::Response> resp) {
    std::unique_lock<std::mutex> lk(mu_);
    if (!route_planner_) {
      resp->success = false;
      resp->error_code = "NO_GRAPH";
      resp->error_msg = "graph not loaded";
      return;
    }
    auto* cc = (collision_checker_ && collision_checker_->hasMap()) ? collision_checker_.get() : nullptr;

    industrial_route_core::RouteResult result;
    if (!req->use_start_pose && !graph_.hasNode(req->start_node_id)) {
      resp->success = false;
      resp->error_code = "BAD_START_NODE";
      resp->error_msg = "start_node_id not found";
      return;
    }
    if (!req->use_goal_pose && !graph_.hasNode(req->goal_node_id)) {
      resp->success = false;
      resp->error_code = "BAD_GOAL_NODE";
      resp->error_msg = "goal_node_id not found";
      return;
    }

    if (req->use_start_pose && req->use_goal_pose) {
      result = route_planner_->planPoseToPose(pose2DFromPoseStamped(req->start),
                                              pose2DFromPoseStamped(req->goal),
                                              planner_opts_, cc);
    } else if (!req->use_start_pose && !req->use_goal_pose) {
      result = route_planner_->planNodeToNode(req->start_node_id, req->goal_node_id, planner_opts_);
    } else {
      // Mixed: resolve missing endpoint to nearest node
      industrial_route_core::Pose2D sp;
      industrial_route_core::Pose2D gp;
      if (req->use_start_pose) sp = pose2DFromPoseStamped(req->start);
      if (req->use_goal_pose) gp = pose2DFromPoseStamped(req->goal);

      std::string s_node = req->start_node_id;
      std::string g_node = req->goal_node_id;

      if (req->use_start_pose) {
        auto nn = industrial_route_core::MapMatching::findNearestNode(graph_, {sp.x, sp.y}, planner_opts_.connect_max_dist);
        if (!nn.success) {
          resp->success = false;
          resp->error_code = "CONNECT_FAILED";
          resp->error_msg = "failed to connect start pose to graph";
          return;
        }
        s_node = nn.node_id;
      }
      if (req->use_goal_pose) {
        auto nn = industrial_route_core::MapMatching::findNearestNode(graph_, {gp.x, gp.y}, planner_opts_.connect_max_dist);
        if (!nn.success) {
          resp->success = false;
          resp->error_code = "CONNECT_FAILED";
          resp->error_msg = "failed to connect goal pose to graph";
          return;
        }
        g_node = nn.node_id;
      }
      result = route_planner_->planNodeToNode(s_node, g_node, planner_opts_);
    }

    resp->success = result.success;
    resp->error_code = result.error_code;
    resp->error_msg = result.error_msg;
    resp->node_sequence.clear();
    resp->node_sequence.reserve(result.node_sequence.size());
    for (const auto& id : result.node_sequence) {
      if (id.rfind("__", 0) == 0) continue;
      resp->node_sequence.push_back(id);
    }
    resp->total_cost = result.total_cost;

    const auto stamp = now();
    resp->dense_path = toPathMsg(result.dense_path, frame_id_, stamp);
    resp->approach_path = toPathMsg(result.approach_path, frame_id_, stamp);
    resp->departure_path = toPathMsg(result.departure_path, frame_id_, stamp);

    if (result.success) {
      last_planned_path_ = result.dense_path;
      path_pub_->publish(resp->dense_path);
    }
  }

  void onGetNearestNode(const std::shared_ptr<GetNearestNode::Request> req,
                        std::shared_ptr<GetNearestNode::Response> resp) {
    std::lock_guard<std::mutex> lk(mu_);
    if (graph_.nodeIds().empty()) {
      resp->success = false;
      return;
    }
    const auto pose = pose2DFromPoseStamped(req->pose);
    const auto nn = industrial_route_core::MapMatching::findNearestNode(graph_, {pose.x, pose.y}, req->max_dist);
    resp->success = nn.success;
    resp->node_id = nn.node_id;
    resp->distance = nn.distance;
  }

  void onUpdateEdge(const std::shared_ptr<UpdateEdge::Request> req,
                    std::shared_ptr<UpdateEdge::Response> resp) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!graph_.hasEdge(req->edge_id)) {
      resp->success = false;
      resp->error_msg = "edge_id not found";
      return;
    }
    try {
      if (req->set_enabled) graph_.setEdgeEnabled(req->edge_id, req->enabled);
      if (req->set_max_speed) graph_.setEdgeMaxSpeed(req->edge_id, req->max_speed);
      if (req->set_bidirectional) graph_.setEdgeBidirectional(req->edge_id, req->bidirectional);
      if (req->set_weight) graph_.setEdgeWeight(req->edge_id, req->weight);
      resp->success = true;
      resp->error_msg = "";
      publishGraph();
    } catch (const std::exception& e) {
      resp->success = false;
      resp->error_msg = e.what();
    }
  }

  void onPause(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    std::lock_guard<std::mutex> lk(exec_mu_);
    if (!active_goal_) {
      resp->success = false;
      resp->message = "no active goal";
      return;
    }
    exec_state_ = req->data ? ExecState::PAUSED : ExecState::EXECUTING;
    resp->success = true;
    resp->message = req->data ? "paused" : "resumed";
  }

  void onCancel(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    std::lock_guard<std::mutex> lk(exec_mu_);
    if (!active_goal_) {
      resp->success = false;
      resp->message = "no active goal";
      return;
    }
    active_goal_->abort(std::make_shared<ExecuteRoute::Result>());
    active_goal_.reset();
    exec_state_ = ExecState::IDLE;
    stopRobot();
    resp->success = true;
    resp->message = "canceled";
  }

  // Action server handlers
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                        std::shared_ptr<const ExecuteRoute::Goal> goal) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!goal->use_path && graph_.nodeIds().empty()) {
      RCLCPP_WARN(get_logger(), "Reject execute: graph not loaded and goal is not a path");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleExecuteRoute> goal_handle) {
    std::lock_guard<std::mutex> lk(exec_mu_);
    if (active_goal_ && active_goal_ == goal_handle) {
      exec_state_ = ExecState::IDLE;
      stopRobot();
      active_goal_.reset();
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleExecuteRoute> goal_handle) {
    std::lock_guard<std::mutex> lk(exec_mu_);
    active_goal_ = goal_handle;
    exec_state_ = ExecState::EXECUTING;
    controller_state_ = PurePursuitController::State{};

    const auto goal = goal_handle->get_goal();
    rotate_in_place_ = goal->rotate_in_place;

    if (goal->use_path) {
      current_path_ = fromPathMsg(goal->path);
    } else if (goal->use_node_sequence) {
      std::lock_guard<std::mutex> glk(mu_);
      current_path_ = industrial_route_core::PathBuilder::buildDensePathFromNodeSequenceBestEffort(
          graph_, goal->node_sequence, planner_opts_);
    } else if (!last_planned_path_.empty()) {
      current_path_ = last_planned_path_;
    } else {
      current_path_.clear();
    }

    controller_->setPath(current_path_);
    last_cmd_time_ = now();
    stopRobot();
  }

  void controlLoop() {
    std::shared_ptr<GoalHandleExecuteRoute> gh;
    ExecState st;
    {
      std::lock_guard<std::mutex> lk(exec_mu_);
      gh = active_goal_;
      st = exec_state_;
    }
    if (!gh) {
      publishStatus("IDLE", 0.0f, "", "");
      return;
    }

    if (st == ExecState::PAUSED) {
      stopRobot();
      publishStatus("PAUSED", controller_state_.progress, "", "");
      return;
    }

    if (gh->is_canceling()) {
      auto res = std::make_shared<ExecuteRoute::Result>();
      res->success = false;
      res->error_code = "CANCELED";
      res->error_msg = "canceled";
      gh->canceled(res);
      {
        std::lock_guard<std::mutex> lk(exec_mu_);
        active_goal_.reset();
        exec_state_ = ExecState::IDLE;
      }
      stopRobot();
      publishStatus("IDLE", 0.0f, "", "");
      return;
    }

    const auto pose = getRobotPose();
    if (!pose) {
      stopRobot();
      publishStatus("ERROR", controller_state_.progress, "", "", "NO_TF", "failed to get robot pose");
      return;
    }

    std::string current_node_id;
    std::string current_edge_id;
    {
      std::lock_guard<std::mutex> lk(mu_);
      const auto nn = industrial_route_core::MapMatching::findNearestNode(graph_, {pose->x, pose->y}, 0.5);
      if (nn.success) current_node_id = nn.node_id;
      const auto ne = industrial_route_core::MapMatching::findNearestEdge(graph_, {pose->x, pose->y}, 0.5);
      if (ne.success) current_edge_id = ne.edge_id;
    }

    last_cmd_time_ = now();

    // In Humble, we don't have direct feedback of current speed; use previous command as proxy.
    const double current_v = last_cmd_.linear.x;

    const auto cmd = controller_->computeCommand(*pose, current_v, rotate_in_place_, &controller_state_);
    last_cmd_ = cmd;
    cmd_pub_->publish(cmd);

    // Feedback
    ExecuteRoute::Feedback fb;
    fb.progress = controller_state_.progress;
    fb.current_pose = poseStampedFromPose2D(*pose, frame_id_, now());
    fb.current_edge_id = current_edge_id;
    fb.current_node_id = current_node_id;
    gh->publish_feedback(std::make_shared<ExecuteRoute::Feedback>(fb));

    publishStatus("EXECUTING", controller_state_.progress, current_edge_id, current_node_id);

    if (controller_state_.reached_goal) {
      stopRobot();
      auto res = std::make_shared<ExecuteRoute::Result>();
      res->success = true;
      res->error_code = "";
      res->error_msg = "";
      gh->succeed(res);
      {
        std::lock_guard<std::mutex> lk(exec_mu_);
        active_goal_.reset();
        exec_state_ = ExecState::IDLE;
      }
      publishStatus("IDLE", 1.0f, current_edge_id, current_node_id);
    }
  }

  void stopRobot() {
    geometry_msgs::msg::Twist z;
    cmd_pub_->publish(z);
  }

  // Parameters
  std::string graph_geojson_path_;
  std::string frame_id_;
  std::string base_frame_id_;
  std::string cmd_vel_topic_;
  std::string map_topic_;
  bool use_map_collision_check_{false};

  industrial_route_core::PlannerOptions planner_opts_;

  // Graph and planner
  std::mutex mu_;
  industrial_route_core::Graph graph_;
  std::unique_ptr<industrial_route_core::RoutePlanner> route_planner_;
  std::unique_ptr<OccupancyGridCollisionChecker> collision_checker_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Publishers / services
  rclcpp::Publisher<industrial_route_interfaces::msg::RouteGraph>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<industrial_route_interfaces::msg::RouteStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
  rclcpp::Service<PlanRoute>::SharedPtr plan_srv_;
  rclcpp::Service<GetNearestNode>::SharedPtr nearest_srv_;
  rclcpp::Service<UpdateEdge>::SharedPtr update_edge_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;

  // Action execution state
  std::mutex exec_mu_;
  ExecState exec_state_{ExecState::IDLE};
  rclcpp_action::Server<ExecuteRoute>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleExecuteRoute> active_goal_;
  bool rotate_in_place_{true};
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::unique_ptr<PurePursuitController> controller_;
  PurePursuitController::State controller_state_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist last_cmd_;

  std::vector<industrial_route_core::Pose2D> current_path_;
  std::vector<industrial_route_core::Pose2D> last_planned_path_;
};

}  // namespace industrial_route_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<industrial_route_ros2::IndustrialRouteServer>());
  rclcpp::shutdown();
  return 0;
}
