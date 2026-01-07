#include "industrial_route_ros2/occupancy_grid_collision_checker.hpp"

#include <algorithm>
#include <cmath>

namespace industrial_route_ros2 {
namespace {

struct GridLinePoint {
  int x;
  int y;
};

std::vector<GridLinePoint> bresenham(int x0, int y0, int x1, int y1) {
  std::vector<GridLinePoint> pts;
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0;
  int y = y0;

  while (true) {
    pts.push_back({x, y});
    if (x == x1 && y == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
  return pts;
}

}  // namespace

OccupancyGridCollisionChecker::OccupancyGridCollisionChecker(rclcpp::Node* node,
                                                             const std::string& topic,
                                                             const Options& opts)
    : opts_(opts) {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic, qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mu_);
        map_ = MapCache{*msg};
      });
}

bool OccupancyGridCollisionChecker::hasMap() const {
  std::lock_guard<std::mutex> lk(mu_);
  return map_.has_value();
}

std::optional<std::pair<int, int>> OccupancyGridCollisionChecker::worldToGrid(double wx, double wy) const {
  if (!map_) return std::nullopt;
  const auto& info = map_->grid.info;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const double res = info.resolution;
  if (res <= 1e-9) return std::nullopt;

  const int mx = static_cast<int>(std::floor((wx - ox) / res));
  const int my = static_cast<int>(std::floor((wy - oy) / res));
  if (mx < 0 || my < 0) return std::nullopt;
  if (mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height)) return std::nullopt;
  return std::make_pair(mx, my);
}

bool OccupancyGridCollisionChecker::cellFree(int mx, int my) const {
  if (!map_) return true;
  const auto& info = map_->grid.info;
  const int w = static_cast<int>(info.width);
  const int h = static_cast<int>(info.height);
  if (mx < 0 || my < 0 || mx >= w || my >= h) return false;

  const std::size_t idx = static_cast<std::size_t>(my * w + mx);
  if (idx >= map_->grid.data.size()) return false;
  const int8_t v = map_->grid.data[idx];
  if (v < 0) return !opts_.treat_unknown_as_occupied;
  return v < opts_.occupied_threshold;
}

bool OccupancyGridCollisionChecker::isLineFree(const industrial_route_core::Point2D& a,
                                               const industrial_route_core::Point2D& b) const {
  std::lock_guard<std::mutex> lk(mu_);
  if (!map_) return true;

  const auto ga = worldToGrid(a.x, a.y);
  const auto gb = worldToGrid(b.x, b.y);
  if (!ga || !gb) return false;

  const auto pts = bresenham(ga->first, ga->second, gb->first, gb->second);
  for (const auto& p : pts) {
    if (!cellFree(p.x, p.y)) return false;
  }
  return true;
}

}  // namespace industrial_route_ros2

