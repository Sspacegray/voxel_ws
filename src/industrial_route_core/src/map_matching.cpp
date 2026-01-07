#include "industrial_route_core/map_matching.hpp"

#include <limits>

namespace industrial_route_core {
namespace {

struct SegmentProjection {
  Point2D projected;
  double t{0.0};       // 0..1 along segment
  double dist{0.0};
  double seg_len{0.0};
};

SegmentProjection projectToSegment(const Point2D& p, const Point2D& a, const Point2D& b) {
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double wx = p.x - a.x;
  const double wy = p.y - a.y;
  const double vv = vx * vx + vy * vy;
  double t = 0.0;
  if (vv > 1e-12) {
    t = (wx * vx + wy * vy) / vv;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
  }
  Point2D proj{a.x + t * vx, a.y + t * vy};
  return {proj, t, distance(p, proj), std::sqrt(vv)};
}

}  // namespace

NearestNodeResult MapMatching::findNearestNode(const Graph& graph, const Point2D& p, double max_dist) {
  NearestNodeResult res;
  res.success = false;
  res.distance = std::numeric_limits<double>::infinity();

  for (const auto& id : graph.nodeIds()) {
    const double d = distance(graph.node(id).position, p);
    if (d < res.distance) {
      res.distance = d;
      res.node_id = id;
    }
  }
  if (res.distance <= max_dist) res.success = true;
  return res;
}

NearestEdgeResult MapMatching::findNearestEdge(const Graph& graph, const Point2D& p, double max_dist) {
  NearestEdgeResult best;
  best.success = false;
  best.distance = std::numeric_limits<double>::infinity();

  for (const auto& eid : graph.edgeIds()) {
    const auto& e = graph.edge(eid);
    if (!e.enabled) continue;
    if (e.polyline.size() < 2) continue;

    double s = 0.0;
    for (std::size_t i = 1; i < e.polyline.size(); ++i) {
      const auto a = e.polyline[i - 1];
      const auto b = e.polyline[i];
      const auto proj = projectToSegment(p, a, b);
      if (proj.dist < best.distance) {
        best.success = true;
        best.edge_id = e.id;
        best.start_id = e.start_id;
        best.end_id = e.end_id;
        best.projected = proj.projected;
        best.distance = proj.dist;
        best.s_along = s + proj.t * proj.seg_len;
        best.edge_length = graph.edgeLength(e);
      }
      s += proj.seg_len;
    }
  }

  if (!best.success || best.distance > max_dist) {
    return NearestEdgeResult{};
  }
  return best;
}

}  // namespace industrial_route_core

