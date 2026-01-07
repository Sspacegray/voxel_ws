#pragma once

#include "industrial_route_core/graph.hpp"

namespace industrial_route_core {

struct PathPlan {
  bool success{false};
  std::string error_msg;
  double total_cost{0.0};
  std::vector<std::string> node_sequence;
  std::vector<std::string> edge_sequence;
};

class Planner {
public:
  static PathPlan dijkstra(const Graph& graph,
                           const std::string& start_node,
                           const std::string& goal_node,
                           const PlannerOptions& opts);
};

}  // namespace industrial_route_core

