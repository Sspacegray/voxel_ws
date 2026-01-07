#include "industrial_route_core/path_builder.hpp"

#include <algorithm>

namespace industrial_route_core {
namespace {

Pose2D poseFromPoints(const Point2D& a, const Point2D& b) {
  Pose2D p;
  p.x = a.x;
  p.y = a.y;
  p.yaw = std::atan2(b.y - a.y, b.x - a.x);
  return p;
}

}  // namespace

std::vector<Pose2D> PathBuilder::densifyPolyline(const std::vector<Point2D>& polyline, double resolution) {
  std::vector<Pose2D> out;
  if (polyline.size() < 2) return out;
  if (resolution <= 1e-3) resolution = 0.2;

  for (std::size_t i = 1; i < polyline.size(); ++i) {
    const auto a = polyline[i - 1];
    const auto b = polyline[i];
    const double seg_len = distance(a, b);
    if (seg_len < 1e-9) continue;

    const int steps = std::max(1, static_cast<int>(std::floor(seg_len / resolution)));
    for (int s = 0; s < steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      Point2D p{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
      out.push_back(poseFromPoints(p, b));
    }
  }
  // Add last point with last yaw
  Pose2D last;
  last.x = polyline.back().x;
  last.y = polyline.back().y;
  last.yaw = out.empty() ? 0.0 : out.back().yaw;
  out.push_back(last);
  return out;
}

std::vector<Pose2D> PathBuilder::buildDensePathFromPlan(const Graph& graph,
                                                        const std::vector<std::string>& node_sequence,
                                                        const std::vector<std::string>& edge_sequence,
                                                        const PlannerOptions& opts) {
  std::vector<Pose2D> out;
  if (node_sequence.size() < 2) return out;
  if (edge_sequence.size() + 1 != node_sequence.size()) return out;

  for (std::size_t i = 0; i < edge_sequence.size(); ++i) {
    const auto& from = node_sequence[i];
    const auto& to = node_sequence[i + 1];
    const auto& eid = edge_sequence[i];
    const auto& e = graph.edge(eid);
    std::vector<Point2D> poly = graph.edgePolylineDirected(e, from, to);

    auto dense = densifyPolyline(poly, opts.densify_resolution);
    if (!out.empty() && !dense.empty()) {
      // Avoid duplicated join point
      dense.erase(dense.begin());
    }
    out.insert(out.end(), dense.begin(), dense.end());
  }
  return out;
}

std::vector<Pose2D> PathBuilder::buildDensePathFromNodeSequenceBestEffort(const Graph& graph,
                                                                          const std::vector<std::string>& node_sequence,
                                                                          const PlannerOptions& opts) {
  std::vector<Pose2D> out;
  if (node_sequence.size() < 2) return out;

  for (std::size_t i = 1; i < node_sequence.size(); ++i) {
    const auto& from = node_sequence[i - 1];
    const auto& to = node_sequence[i];
    if (!graph.hasNode(from) || !graph.hasNode(to)) return {};

    const Edge* edge_ptr = nullptr;
    std::string edge_id;
    bool reversed = false;

    for (const auto& eid : graph.edgeIds()) {
      const auto& e = graph.edge(eid);
      if (!e.enabled) continue;
      if (e.start_id == from && e.end_id == to) {
        edge_ptr = &e;
        edge_id = eid;
        reversed = false;
        break;
      }
      if (e.bidirectional && e.start_id == to && e.end_id == from) {
        edge_ptr = &e;
        edge_id = eid;
        reversed = true;
        break;
      }
    }

    std::vector<Point2D> poly;
    if (edge_ptr) {
      poly = edge_ptr->polyline;
      if (reversed) std::reverse(poly.begin(), poly.end());
    } else {
      poly = {graph.node(from).position, graph.node(to).position};
    }

    auto dense = densifyPolyline(poly, opts.densify_resolution);
    if (!out.empty() && !dense.empty()) dense.erase(dense.begin());
    out.insert(out.end(), dense.begin(), dense.end());
  }
  return out;
}

}  // namespace industrial_route_core
