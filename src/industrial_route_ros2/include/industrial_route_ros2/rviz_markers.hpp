#pragma once

#include "industrial_route_core/graph.hpp"
#include "industrial_route_interfaces/msg/route_graph.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace industrial_route_ros2 {

class RvizMarkers {
public:
  static visualization_msgs::msg::MarkerArray makeGraphMarkers(const industrial_route_core::Graph& graph,
                                                               const std::string& frame_id);
  static industrial_route_interfaces::msg::RouteGraph toMsg(const industrial_route_core::Graph& graph,
                                                           const std::string& frame_id);
};

}  // namespace industrial_route_ros2

