#include "industrial_route_core/route_planner.hpp"

#include "industrial_route_core/path_builder.hpp"

#include <algorithm>
#include <limits>

namespace industrial_route_core {
namespace {

std::vector<std::string> kNearestNodes(const Graph& graph, const Point2D& p, std::size_t k, double max_dist) {
  struct Item {
    std::string id;
    double d;
  };
  std::vector<Item> items;
  for (const auto& id : graph.nodeIds()) {
    const double d = distance(graph.node(id).position, p);
    if (d <= max_dist) items.push_back({id, d});
  }
  std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.d < b.d; });
  if (items.size() > k) items.resize(k);
  std::vector<std::string> ids;
  ids.reserve(items.size());
  for (const auto& it : items) ids.push_back(it.id);
  return ids;
}

std::vector<Pose2D> straightConnector(const Point2D& from, const Point2D& to, double resolution) {
  std::vector<Point2D> poly{{from.x, from.y}, to};
  return PathBuilder::densifyPolyline(poly, resolution);
}

std::vector<Point2D> polylineSegmentBetween(const std::vector<Point2D>& polyline,
                                            double s_from,
                                            const Point2D& p_from,
                                            double s_to,
                                            const Point2D& p_to) {
  if (polyline.size() < 2) return {};
  const bool forward = s_from <= s_to;
  const double s_min = std::min(s_from, s_to);
  const double s_max = std::max(s_from, s_to);

  std::vector<Point2D> seg;
  seg.push_back(forward ? p_from : p_to);

  double s = 0.0;
  for (std::size_t i = 1; i < polyline.size(); ++i) {
    const auto a = polyline[i - 1];
    const auto b = polyline[i];
    const double len = distance(a, b);
    const double s_next = s + len;

    if (s_next < s_min) {
      s = s_next;
      continue;
    }
    if (s > s_max) break;

    // Include intermediate vertex if it lies within [s_min, s_max]
    if (s >= s_min && s_next <= s_max) {
      seg.push_back(b);
    } else if (s < s_max && s_next > s_max) {
      // segment crosses s_max: append end projection and stop
      seg.push_back(forward ? p_to : p_from);
      break;
    }
    s = s_next;
  }

  if (seg.size() < 2) {
    seg.push_back(forward ? p_to : p_from);
  }
  if (!forward) {
    std::reverse(seg.begin(), seg.end());
  }
  return seg;
}

}  // namespace

bool RoutePlanner::isConnectorFree(const CollisionChecker* collision_checker, const Point2D& a, const Point2D& b) {
  if (!collision_checker) return true;
  return collision_checker->isLineFree(a, b);
}

std::pair<std::vector<Point2D>, std::vector<Point2D>> RoutePlanner::splitPolylineAtProjection(
    const std::vector<Point2D>& polyline, const Point2D& projected) {
  if (polyline.size() < 2) return {{}, {}};
  // Find the segment where projection lies (minimum distance).
  double best_d = std::numeric_limits<double>::infinity();
  std::size_t best_i = 1;
  double best_t = 0.0;
  for (std::size_t i = 1; i < polyline.size(); ++i) {
    const auto a = polyline[i - 1];
    const auto b = polyline[i];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double vv = vx * vx + vy * vy;
    double t = 0.0;
    if (vv > 1e-12) {
      t = ((projected.x - a.x) * vx + (projected.y - a.y) * vy) / vv;
      if (t < 0.0) t = 0.0;
      if (t > 1.0) t = 1.0;
    }
    const Point2D proj{a.x + t * vx, a.y + t * vy};
    const double d = distance(proj, projected);
    if (d < best_d) {
      best_d = d;
      best_i = i;
      best_t = t;
    }
  }

  std::vector<Point2D> start_to_proj;
  std::vector<Point2D> proj_to_end;

  start_to_proj.insert(start_to_proj.end(), polyline.begin(), polyline.begin() + static_cast<long>(best_i));
  proj_to_end.insert(proj_to_end.end(), polyline.begin() + static_cast<long>(best_i), polyline.end());

  // Insert the exact projected point between
  const auto a = polyline[best_i - 1];
  const auto b = polyline[best_i];
  const Point2D exact{a.x + best_t * (b.x - a.x), a.y + best_t * (b.y - a.y)};

  start_to_proj.push_back(exact);
  proj_to_end.insert(proj_to_end.begin(), exact);

  return {start_to_proj, proj_to_end};
}

RoutePlanner::AnchorBuildResult RoutePlanner::addStartAnchor(Graph* tmp_graph,
                                                            const Pose2D& start_pose,
                                                            const PlannerOptions& opts,
                                                            const CollisionChecker* collision_checker) const {
  AnchorBuildResult out;
  out.anchor_node_id = "__start_anchor__";
  const Point2D start_p{start_pose.x, start_pose.y};

  const auto edge_match = MapMatching::findNearestEdge(*graph_, start_p, opts.on_graph_max_dist);
  if (edge_match.success && isConnectorFree(collision_checker, start_p, edge_match.projected)) {
    Node anchor;
    anchor.id = out.anchor_node_id;
    anchor.position = edge_match.projected;
    tmp_graph->addNode(std::move(anchor));
    out.used_edge_projection = true;
    out.projected_edge_id = edge_match.edge_id;
    out.projected_s_along = edge_match.s_along;
    out.projected_point = edge_match.projected;
    out.approach_path = straightConnector(start_p, edge_match.projected, opts.densify_resolution);
    addEdgeProjectionConnectionsForStart(tmp_graph, out.anchor_node_id, edge_match, opts);
    return out;
  }

  Node anchor;
  anchor.id = out.anchor_node_id;
  anchor.position = start_p;
  tmp_graph->addNode(std::move(anchor));
  addOffGraphConnectionsFromStart(tmp_graph, out.anchor_node_id, start_pose, opts, collision_checker);
  return out;
}

RoutePlanner::AnchorBuildResult RoutePlanner::addGoalAnchor(Graph* tmp_graph,
                                                           const Pose2D& goal_pose,
                                                           const PlannerOptions& opts,
                                                           const CollisionChecker* collision_checker) const {
  AnchorBuildResult out;
  out.anchor_node_id = "__goal_anchor__";
  const Point2D goal_p{goal_pose.x, goal_pose.y};

  const auto edge_match = MapMatching::findNearestEdge(*graph_, goal_p, opts.on_graph_max_dist);
  if (edge_match.success && isConnectorFree(collision_checker, goal_p, edge_match.projected)) {
    Node anchor;
    anchor.id = out.anchor_node_id;
    anchor.position = edge_match.projected;
    tmp_graph->addNode(std::move(anchor));
    out.used_edge_projection = true;
    out.projected_edge_id = edge_match.edge_id;
    out.projected_s_along = edge_match.s_along;
    out.projected_point = edge_match.projected;
    out.departure_path = straightConnector(edge_match.projected, goal_p, opts.densify_resolution);
    addEdgeProjectionConnectionsForGoal(tmp_graph, out.anchor_node_id, edge_match, opts);
    return out;
  }

  Node anchor;
  anchor.id = out.anchor_node_id;
  anchor.position = goal_p;
  tmp_graph->addNode(std::move(anchor));
  addOffGraphConnectionsToGoal(tmp_graph, out.anchor_node_id, goal_pose, opts, collision_checker);
  return out;
}

void RoutePlanner::addOffGraphConnectionsFromStart(Graph* tmp_graph,
                                                   const std::string& start_anchor_id,
                                                   const Pose2D& start_pose,
                                                   const PlannerOptions& opts,
                                                   const CollisionChecker* collision_checker) const {
  const Point2D p{start_pose.x, start_pose.y};
  for (const auto& nid : kNearestNodes(*graph_, p, opts.connect_k_nearest, opts.connect_max_dist)) {
    const auto& n = graph_->node(nid);
    if (!isConnectorFree(collision_checker, p, n.position)) continue;
    Edge e;
    e.id = start_anchor_id + "_to_" + nid;
    e.start_id = start_anchor_id;
    e.end_id = nid;
    e.enabled = true;
    e.bidirectional = false;
    e.weight = 1.0;
    e.max_speed = opts.use_max_speed_cost ? 0.6 : 0.6;
    e.polyline = {p, n.position};
    tmp_graph->addEdge(std::move(e));
  }
}

void RoutePlanner::addOffGraphConnectionsToGoal(Graph* tmp_graph,
                                                const std::string& goal_anchor_id,
                                                const Pose2D& goal_pose,
                                                const PlannerOptions& opts,
                                                const CollisionChecker* collision_checker) const {
  const Point2D p{goal_pose.x, goal_pose.y};
  for (const auto& nid : kNearestNodes(*graph_, p, opts.connect_k_nearest, opts.connect_max_dist)) {
    const auto& n = graph_->node(nid);
    if (!isConnectorFree(collision_checker, n.position, p)) continue;
    Edge e;
    e.id = nid + "_to_" + goal_anchor_id;
    e.start_id = nid;
    e.end_id = goal_anchor_id;
    e.enabled = true;
    e.bidirectional = false;
    e.weight = 1.0;
    e.max_speed = opts.use_max_speed_cost ? 0.6 : 0.6;
    e.polyline = {n.position, p};
    tmp_graph->addEdge(std::move(e));
  }
}

void RoutePlanner::addEdgeProjectionConnectionsForStart(Graph* tmp_graph,
                                                        const std::string& start_anchor_id,
                                                        const NearestEdgeResult& edge_match,
                                                        const PlannerOptions& opts) const {
  (void)opts;
  const auto& orig = graph_->edge(edge_match.edge_id);
  auto [start_to_proj, proj_to_end] = splitPolylineAtProjection(orig.polyline, edge_match.projected);

  // anchor (proj) -> start node : reverse(start_to_proj)
  Edge e1;
  e1.id = start_anchor_id + "_to_" + orig.start_id + "_via_" + orig.id;
  e1.start_id = start_anchor_id;
  e1.end_id = orig.start_id;
  e1.enabled = orig.enabled;
  e1.bidirectional = false;
  e1.weight = orig.weight;
  e1.max_speed = orig.max_speed;
  e1.polyline = start_to_proj;
  std::reverse(e1.polyline.begin(), e1.polyline.end());
  tmp_graph->addEdge(std::move(e1));

  // anchor (proj) -> end node : proj_to_end
  Edge e2;
  e2.id = start_anchor_id + "_to_" + orig.end_id + "_via_" + orig.id;
  e2.start_id = start_anchor_id;
  e2.end_id = orig.end_id;
  e2.enabled = orig.enabled;
  e2.bidirectional = false;
  e2.weight = orig.weight;
  e2.max_speed = orig.max_speed;
  e2.polyline = proj_to_end;
  tmp_graph->addEdge(std::move(e2));
}

void RoutePlanner::addEdgeProjectionConnectionsForGoal(Graph* tmp_graph,
                                                       const std::string& goal_anchor_id,
                                                       const NearestEdgeResult& edge_match,
                                                       const PlannerOptions& opts) const {
  (void)opts;
  const auto& orig = graph_->edge(edge_match.edge_id);
  auto [start_to_proj, proj_to_end] = splitPolylineAtProjection(orig.polyline, edge_match.projected);

  // start node -> goal anchor (proj) : start_to_proj
  Edge e1;
  e1.id = orig.start_id + "_to_" + goal_anchor_id + "_via_" + orig.id;
  e1.start_id = orig.start_id;
  e1.end_id = goal_anchor_id;
  e1.enabled = orig.enabled;
  e1.bidirectional = false;
  e1.weight = orig.weight;
  e1.max_speed = orig.max_speed;
  e1.polyline = start_to_proj;
  tmp_graph->addEdge(std::move(e1));

  // end node -> goal anchor (proj) : reverse(proj_to_end)
  Edge e2;
  e2.id = orig.end_id + "_to_" + goal_anchor_id + "_via_" + orig.id;
  e2.start_id = orig.end_id;
  e2.end_id = goal_anchor_id;
  e2.enabled = orig.enabled;
  e2.bidirectional = false;
  e2.weight = orig.weight;
  e2.max_speed = orig.max_speed;
  e2.polyline = proj_to_end;
  std::reverse(e2.polyline.begin(), e2.polyline.end());
  tmp_graph->addEdge(std::move(e2));
}

RouteResult RoutePlanner::planNodeToNode(const std::string& start_node, const std::string& goal_node, const PlannerOptions& opts) const {
  RouteResult res;
  if (!graph_) {
    res.success = false;
    res.error_code = "NO_GRAPH";
    res.error_msg = "graph is null";
    return res;
  }
  const auto plan = Planner::dijkstra(*graph_, start_node, goal_node, opts);
  if (!plan.success) {
    res.success = false;
    res.error_code = "NO_PATH";
    res.error_msg = plan.error_msg;
    return res;
  }
  res.success = true;
  res.node_sequence = plan.node_sequence;
  res.dense_path = PathBuilder::buildDensePathFromPlan(*graph_, plan.node_sequence, plan.edge_sequence, opts);
  res.total_cost = plan.total_cost;
  return res;
}

RouteResult RoutePlanner::planPoseToPose(const Pose2D& start,
                                        const Pose2D& goal,
                                        const PlannerOptions& opts,
                                        const CollisionChecker* collision_checker) const {
  RouteResult best;
  best.success = false;
  best.error_code = "NO_PATH";
  best.error_msg = "no feasible route";

  if (!graph_) {
    best.error_code = "NO_GRAPH";
    best.error_msg = "graph is null";
    return best;
  }

  Graph tmp = *graph_;
  const auto start_anchor = addStartAnchor(&tmp, start, opts, collision_checker);
  const auto goal_anchor = addGoalAnchor(&tmp, goal, opts, collision_checker);

  // If both anchors project onto the same edge, add a direct segment between projections
  // to avoid detouring via endpoints.
  if (start_anchor.used_edge_projection && goal_anchor.used_edge_projection &&
      !start_anchor.projected_edge_id.empty() &&
      start_anchor.projected_edge_id == goal_anchor.projected_edge_id) {
    const auto& e = graph_->edge(start_anchor.projected_edge_id);
    const auto seg = polylineSegmentBetween(e.polyline,
                                           start_anchor.projected_s_along,
                                           start_anchor.projected_point,
                                           goal_anchor.projected_s_along,
                                           goal_anchor.projected_point);
    if (seg.size() >= 2) {
      const bool forward = start_anchor.projected_s_along <= goal_anchor.projected_s_along;
      if (forward || e.bidirectional) {
        Edge direct;
        direct.id = start_anchor.anchor_node_id + "_to_" + goal_anchor.anchor_node_id + "_on_" + e.id;
        direct.start_id = start_anchor.anchor_node_id;
        direct.end_id = goal_anchor.anchor_node_id;
        direct.enabled = e.enabled;
        direct.bidirectional = false;
        direct.weight = e.weight;
        direct.max_speed = e.max_speed;
        direct.polyline = seg;
        tmp.addEdge(std::move(direct));
      }
      if ((!forward) || e.bidirectional) {
        Edge direct_rev;
        direct_rev.id = goal_anchor.anchor_node_id + "_to_" + start_anchor.anchor_node_id + "_on_" + e.id;
        direct_rev.start_id = goal_anchor.anchor_node_id;
        direct_rev.end_id = start_anchor.anchor_node_id;
        direct_rev.enabled = e.enabled;
        direct_rev.bidirectional = false;
        direct_rev.weight = e.weight;
        direct_rev.max_speed = e.max_speed;
        direct_rev.polyline = seg;
        std::reverse(direct_rev.polyline.begin(), direct_rev.polyline.end());
        tmp.addEdge(std::move(direct_rev));
      }
    }
  }

  const auto plan = Planner::dijkstra(tmp, start_anchor.anchor_node_id, goal_anchor.anchor_node_id, opts);
  if (!plan.success) {
    best.success = false;
    best.error_code = "NO_PATH";
    best.error_msg = plan.error_msg;
    return best;
  }

  auto dense_between = PathBuilder::buildDensePathFromPlan(tmp, plan.node_sequence, plan.edge_sequence, opts);
  if (dense_between.empty()) {
    best.success = false;
    best.error_code = "BUILD_FAILED";
    best.error_msg = "failed to build dense path";
    return best;
  }

  best.success = true;
  best.error_code.clear();
  best.error_msg.clear();
  best.node_sequence = plan.node_sequence;
  best.total_cost = plan.total_cost;
  best.approach_path = start_anchor.approach_path;
  best.departure_path = goal_anchor.departure_path;

  // Stitch final path: (start pose -> start anchor) + (anchor->anchor) + (goal anchor -> goal pose)
  best.dense_path = dense_between;
  if (!best.approach_path.empty()) {
    // approach ends at anchor; dense starts at anchor
    best.approach_path.pop_back();
    std::vector<Pose2D> stitched;
    stitched.reserve(best.approach_path.size() + best.dense_path.size() + best.departure_path.size());
    stitched.insert(stitched.end(), best.approach_path.begin(), best.approach_path.end());
    stitched.insert(stitched.end(), best.dense_path.begin(), best.dense_path.end());
    best.dense_path = std::move(stitched);
  }
  if (!best.departure_path.empty()) {
    // dense ends at anchor; departure starts at anchor
    best.departure_path.erase(best.departure_path.begin());
    best.dense_path.insert(best.dense_path.end(), best.departure_path.begin(), best.departure_path.end());
  }
  return best;
}

}  // namespace industrial_route_core
