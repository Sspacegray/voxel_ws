#pragma once

#include <mutex>
#include <optional>

#include "industrial_route_core/types.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace industrial_route_ros2 {

class OccupancyGridCollisionChecker final : public industrial_route_core::CollisionChecker {
public:
  struct Options {
    int occupied_threshold{50};
    bool treat_unknown_as_occupied{true};
  };

  OccupancyGridCollisionChecker(rclcpp::Node* node,
                                const std::string& topic,
                                const Options& opts);

  bool isLineFree(const industrial_route_core::Point2D& a,
                  const industrial_route_core::Point2D& b) const override;

  bool hasMap() const;

private:
  struct MapCache {
    nav_msgs::msg::OccupancyGrid grid;
  };

  std::optional<std::pair<int, int>> worldToGrid(double wx, double wy) const;
  bool cellFree(int mx, int my) const;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  Options opts_;
  mutable std::mutex mu_;
  std::optional<MapCache> map_;
};

}  // namespace industrial_route_ros2

