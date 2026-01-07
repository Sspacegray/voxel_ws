#pragma once

#include "industrial_route_core/graph.hpp"

namespace industrial_route_core {

struct NearestNodeResult {
  bool success{false};
  std::string node_id;
  double distance{0.0};
};

struct NearestEdgeResult {
  bool success{false};
  std::string edge_id;
  std::string start_id;
  std::string end_id;
  Point2D projected;
  double distance{0.0};
  double s_along{0.0};     // meters from start_id along directed polyline
  double edge_length{0.0};
};

class MapMatching {
public:
  static NearestNodeResult findNearestNode(const Graph& graph, const Point2D& p, double max_dist);
  static NearestEdgeResult findNearestEdge(const Graph& graph, const Point2D& p, double max_dist);
};

}  // namespace industrial_route_core

