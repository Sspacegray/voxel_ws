#pragma once

#include "industrial_route_core/graph.hpp"

namespace industrial_route_core {

class PathBuilder {
public:
  static std::vector<Pose2D> densifyPolyline(const std::vector<Point2D>& polyline, double resolution);
  static std::vector<Pose2D> buildDensePathFromPlan(const Graph& graph,
                                                    const std::vector<std::string>& node_sequence,
                                                    const std::vector<std::string>& edge_sequence,
                                                    const PlannerOptions& opts);
  static std::vector<Pose2D> buildDensePathFromNodeSequenceBestEffort(const Graph& graph,
                                                                      const std::vector<std::string>& node_sequence,
                                                                      const PlannerOptions& opts);
};

}  // namespace industrial_route_core
