#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace industrial_route_core {

struct Point2D {
  double x{0.0};
  double y{0.0};
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double distance(const Point2D& a, const Point2D& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double normalizeYaw(double yaw) {
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

inline double shortestAngularDistance(double from, double to) {
  return normalizeYaw(to - from);
}

struct Node {
  std::string id;
  Point2D position;
  std::optional<double> theta;
  std::string type{"NORMAL"};
  std::unordered_map<std::string, std::string> properties;
};

struct Edge {
  std::string id;
  std::string start_id;
  std::string end_id;
  bool bidirectional{false};
  bool enabled{true};
  double weight{1.0};
  double max_speed{0.6};  // m/s, conservative default for indoor diff-drive
  std::vector<Point2D> polyline;
  std::unordered_map<std::string, std::string> properties;
};

struct EdgeTraversal {
  std::string edge_id;
  std::string from_id;
  std::string to_id;
  double length{0.0};
  double cost{0.0};
};

struct RouteResult {
  bool success{false};
  std::string error_code;
  std::string error_msg;
  std::vector<std::string> node_sequence;
  std::vector<Pose2D> dense_path;
  std::vector<Pose2D> approach_path;
  std::vector<Pose2D> departure_path;
  double total_cost{0.0};
};

struct PlannerOptions {
  double connect_max_dist{3.0};     // meters
  std::size_t connect_k_nearest{5}; // candidates
  double on_graph_max_dist{0.5};    // meters
  double densify_resolution{0.2};   // meters
  bool use_edge_weight{true};
  bool use_max_speed_cost{false};   // if true: cost = length / max_speed
};

class CollisionChecker {
public:
  virtual ~CollisionChecker() = default;
  virtual bool isLineFree(const Point2D& a, const Point2D& b) const = 0;
};

}  // namespace industrial_route_core

