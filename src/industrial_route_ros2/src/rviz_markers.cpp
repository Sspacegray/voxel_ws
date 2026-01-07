#include "industrial_route_ros2/rviz_markers.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace industrial_route_ros2 {
namespace {

geometry_msgs::msg::Quaternion yawToQuat(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion out;
  out = tf2::toMsg(q);
  return out;
}

std::pair<std::vector<std::string>, std::vector<std::string>> kvToArrays(
    const std::unordered_map<std::string, std::string>& kv) {
  std::vector<std::string> keys;
  std::vector<std::string> vals;
  keys.reserve(kv.size());
  vals.reserve(kv.size());
  for (const auto& [k, v] : kv) {
    keys.push_back(k);
    vals.push_back(v);
  }
  return {keys, vals};
}

}  // namespace

visualization_msgs::msg::MarkerArray RvizMarkers::makeGraphMarkers(const industrial_route_core::Graph& graph,
                                                                   const std::string& frame_id) {
  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker nodes;
  nodes.header.frame_id = frame_id;
  nodes.header.stamp = rclcpp::Clock().now();
  nodes.ns = "industrial_route_nodes";
  nodes.id = 1;
  nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes.action = visualization_msgs::msg::Marker::ADD;
  nodes.scale.x = 0.15;
  nodes.scale.y = 0.15;
  nodes.scale.z = 0.15;
  nodes.color.a = 1.0;
  nodes.color.r = 0.1;
  nodes.color.g = 0.8;
  nodes.color.b = 0.2;

  for (const auto& nid : graph.nodeIds()) {
    const auto& n = graph.node(nid);
    geometry_msgs::msg::Point p;
    p.x = n.position.x;
    p.y = n.position.y;
    p.z = 0.0;
    nodes.points.push_back(p);
  }
  arr.markers.push_back(nodes);

  visualization_msgs::msg::Marker edges;
  edges.header.frame_id = frame_id;
  edges.header.stamp = nodes.header.stamp;
  edges.ns = "industrial_route_edges";
  edges.id = 2;
  edges.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges.action = visualization_msgs::msg::Marker::ADD;
  edges.scale.x = 0.05;
  edges.color.a = 0.9;
  edges.color.r = 0.2;
  edges.color.g = 0.6;
  edges.color.b = 1.0;

  for (const auto& eid : graph.edgeIds()) {
    const auto& e = graph.edge(eid);
    if (e.polyline.size() < 2) continue;
    for (std::size_t i = 1; i < e.polyline.size(); ++i) {
      geometry_msgs::msg::Point a;
      a.x = e.polyline[i - 1].x;
      a.y = e.polyline[i - 1].y;
      geometry_msgs::msg::Point b;
      b.x = e.polyline[i].x;
      b.y = e.polyline[i].y;
      edges.points.push_back(a);
      edges.points.push_back(b);
    }
  }
  arr.markers.push_back(edges);

  return arr;
}

industrial_route_interfaces::msg::RouteGraph RvizMarkers::toMsg(const industrial_route_core::Graph& graph,
                                                                const std::string& frame_id) {
  industrial_route_interfaces::msg::RouteGraph out;
  out.header.frame_id = frame_id;
  out.header.stamp = rclcpp::Clock().now();

  for (const auto& nid : graph.nodeIds()) {
    const auto& n = graph.node(nid);
    industrial_route_interfaces::msg::RouteNode msg;
    msg.id = n.id;
    msg.pose.position.x = n.position.x;
    msg.pose.position.y = n.position.y;
    msg.pose.position.z = 0.0;
    msg.pose.orientation = yawToQuat(n.theta.value_or(0.0));
    msg.type = n.type;
    auto [keys, vals] = kvToArrays(n.properties);
    msg.property_keys = std::move(keys);
    msg.property_values = std::move(vals);
    out.nodes.push_back(std::move(msg));
  }

  for (const auto& eid : graph.edgeIds()) {
    const auto& e = graph.edge(eid);
    industrial_route_interfaces::msg::RouteEdge msg;
    msg.id = e.id;
    msg.start_id = e.start_id;
    msg.end_id = e.end_id;
    msg.bidirectional = e.bidirectional;
    msg.enabled = e.enabled;
    msg.weight = e.weight;
    msg.max_speed = e.max_speed;
    for (const auto& pt : e.polyline) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.0;
      msg.polyline.push_back(p);
    }
    auto [keys, vals] = kvToArrays(e.properties);
    msg.property_keys = std::move(keys);
    msg.property_values = std::move(vals);
    out.edges.push_back(std::move(msg));
  }

  return out;
}

}  // namespace industrial_route_ros2

