#pragma once

#include "industrial_route_core/graph.hpp"
#include "industrial_route_core/map_matching.hpp"
#include "industrial_route_core/planner.hpp"
#include "industrial_route_core/types.hpp"

namespace industrial_route_core {

class RoutePlanner {
public:
  explicit RoutePlanner(const Graph* graph) : graph_(graph) {}

  RouteResult planPoseToPose(const Pose2D& start,
                            const Pose2D& goal,
                            const PlannerOptions& opts,
                            const CollisionChecker* collision_checker = nullptr) const;

  RouteResult planNodeToNode(const std::string& start_node,
                            const std::string& goal_node,
                            const PlannerOptions& opts) const;

private:
  struct AnchorBuildResult {
    std::string anchor_node_id;
    std::vector<Pose2D> approach_path;
    std::vector<Pose2D> departure_path;
    bool used_edge_projection{false};
    std::string projected_edge_id;
    double projected_s_along{0.0};
    Point2D projected_point;
  };

  AnchorBuildResult addStartAnchor(Graph* tmp_graph,
                                  const Pose2D& start_pose,
                                  const PlannerOptions& opts,
                                  const CollisionChecker* collision_checker) const;

  AnchorBuildResult addGoalAnchor(Graph* tmp_graph,
                                 const Pose2D& goal_pose,
                                 const PlannerOptions& opts,
                                 const CollisionChecker* collision_checker) const;

  void addOffGraphConnectionsFromStart(Graph* tmp_graph,
                                      const std::string& start_anchor_id,
                                      const Pose2D& start_pose,
                                      const PlannerOptions& opts,
                                      const CollisionChecker* collision_checker) const;

  void addOffGraphConnectionsToGoal(Graph* tmp_graph,
                                   const std::string& goal_anchor_id,
                                   const Pose2D& goal_pose,
                                   const PlannerOptions& opts,
                                   const CollisionChecker* collision_checker) const;

  void addEdgeProjectionConnectionsForStart(Graph* tmp_graph,
                                           const std::string& start_anchor_id,
                                           const NearestEdgeResult& edge_match,
                                           const PlannerOptions& opts) const;

  void addEdgeProjectionConnectionsForGoal(Graph* tmp_graph,
                                          const std::string& goal_anchor_id,
                                          const NearestEdgeResult& edge_match,
                                          const PlannerOptions& opts) const;

  static bool isConnectorFree(const CollisionChecker* collision_checker, const Point2D& a, const Point2D& b);
  static std::pair<std::vector<Point2D>, std::vector<Point2D>> splitPolylineAtProjection(const std::vector<Point2D>& polyline,
                                                                                         const Point2D& projected);

  const Graph* graph_{nullptr};
};

}  // namespace industrial_route_core
