#pragma once

#include <Eigen/Core>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace ddr_navigation
{
class CostmapAdapter
{
public:
  CostmapAdapter(nav2_costmap_2d::Costmap2D * costmap, bool unknown_is_free);

  void setCostmap(nav2_costmap_2d::Costmap2D * costmap);

  void updateDistanceField();

  inline int Index2Vectornum(int x, int y) const
  {
    return y * GLX_SIZE_ + x;
  }

  bool worldToMap(double wx, double wy, int & mx, int & my) const;

  Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d & coord) const;
  Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i & index) const;

  double getDistanceReal(const Eigen::Vector2i & index) const;
  double getDistanceReal(const Eigen::Vector2d & coord) const;

  bool isOccWithSafeDis(int x, int y, double safe_dis) const;
  bool isUnOccupied(int x, int y) const;
  bool isOccupied(int x, int y) const;

  int GLX_SIZE_ = 0;
  int GLY_SIZE_ = 0;

private:
  void refreshMetadata();
  bool isWithinBounds(int x, int y) const;

  nav2_costmap_2d::Costmap2D * costmap_;
  bool unknown_is_free_;
  double resolution_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  int size_x_ = 0;
  int size_y_ = 0;
  std::vector<double> distance_field_;
};
}  // namespace ddr_navigation
