#include "ddr_jps_global_planner/costmap_adapter.hpp"

#include <cmath>
#include <limits>
#include <queue>

namespace ddr_navigation
{
namespace
{
struct DistanceNode
{
  int x;
  int y;
  double distance;
};

struct DistanceCompare
{
  bool operator()(const DistanceNode & lhs, const DistanceNode & rhs) const
  {
    return lhs.distance > rhs.distance;
  }
};
}  // namespace

CostmapAdapter::CostmapAdapter(nav2_costmap_2d::Costmap2D * costmap, bool unknown_is_free)
: costmap_(costmap), unknown_is_free_(unknown_is_free)
{
  refreshMetadata();
  updateDistanceField();
}

void CostmapAdapter::setCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  costmap_ = costmap;
  refreshMetadata();
}

void CostmapAdapter::refreshMetadata()
{
  if (costmap_ == nullptr) {
    size_x_ = size_y_ = GLX_SIZE_ = GLY_SIZE_ = 0;
    resolution_ = 0.05;
    origin_x_ = origin_y_ = 0.0;
    distance_field_.clear();
    return;
  }

  size_x_ = static_cast<int>(costmap_->getSizeInCellsX());
  size_y_ = static_cast<int>(costmap_->getSizeInCellsY());
  GLX_SIZE_ = size_x_;
  GLY_SIZE_ = size_y_;
  resolution_ = costmap_->getResolution();
  origin_x_ = costmap_->getOriginX();
  origin_y_ = costmap_->getOriginY();
  distance_field_.assign(GLX_SIZE_ * GLY_SIZE_, std::numeric_limits<double>::infinity());
}

bool CostmapAdapter::isWithinBounds(int x, int y) const
{
  return x >= 0 && x < GLX_SIZE_ && y >= 0 && y < GLY_SIZE_;
}

void CostmapAdapter::updateDistanceField()
{
  if (costmap_ == nullptr) {
    return;
  }

  refreshMetadata();

  using Queue = std::priority_queue<DistanceNode, std::vector<DistanceNode>, DistanceCompare>;
  Queue queue;

  for (int y = 0; y < GLY_SIZE_; ++y) {
    for (int x = 0; x < GLX_SIZE_; ++x) {
      if (isOccupied(x, y)) {
        const int idx = Index2Vectornum(x, y);
        distance_field_[idx] = 0.0;
        queue.push(DistanceNode{x, y, 0.0});
      }
    }
  }

  const double straight = resolution_;
  const double diagonal = resolution_ * std::sqrt(2.0);
  constexpr int directions[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  while (!queue.empty()) {
    const auto current = queue.top();
    queue.pop();
    const int current_idx = Index2Vectornum(current.x, current.y);
    if (current.distance > distance_field_[current_idx] + 1e-6) {
      continue;
    }

    for (const auto & dir : directions) {
      const int nx = current.x + dir[0];
      const int ny = current.y + dir[1];
      if (!isWithinBounds(nx, ny)) {
        continue;
      }
      const double step = (dir[0] == 0 || dir[1] == 0) ? straight : diagonal;
      const double nd = current.distance + step;
      const int nidx = Index2Vectornum(nx, ny);
      if (nd < distance_field_[nidx]) {
        distance_field_[nidx] = nd;
        queue.push(DistanceNode{nx, ny, nd});
      }
    }
  }
}

bool CostmapAdapter::worldToMap(double wx, double wy, int & mx, int & my) const
{
  if (costmap_ == nullptr) {
    return false;
  }
  unsigned int umx = 0;
  unsigned int umy = 0;
  if (!costmap_->worldToMap(wx, wy, umx, umy)) {
    return false;
  }
  mx = static_cast<int>(umx);
  my = static_cast<int>(umy);
  return true;
}

Eigen::Vector2i CostmapAdapter::coord2gridIndex(const Eigen::Vector2d & coord) const
{
  int mx = -1;
  int my = -1;
  if (!worldToMap(coord.x(), coord.y(), mx, my)) {
    return Eigen::Vector2i(-1, -1);
  }
  return Eigen::Vector2i(mx, my);
}

Eigen::Vector2d CostmapAdapter::gridIndex2coordd(const Eigen::Vector2i & index) const
{
  const double wx = origin_x_ + (static_cast<double>(index.x()) + 0.5) * resolution_;
  const double wy = origin_y_ + (static_cast<double>(index.y()) + 0.5) * resolution_;
  return Eigen::Vector2d(wx, wy);
}

double CostmapAdapter::getDistanceReal(const Eigen::Vector2i & index) const
{
  if (!isWithinBounds(index.x(), index.y()) || distance_field_.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  return distance_field_[Index2Vectornum(index.x(), index.y())];
}

double CostmapAdapter::getDistanceReal(const Eigen::Vector2d & coord) const
{
  const auto idx = coord2gridIndex(coord);
  return getDistanceReal(idx);
}

bool CostmapAdapter::isOccWithSafeDis(int x, int y, double safe_dis) const
{
  if (!isWithinBounds(x, y)) {
    return true;
  }
  if (distance_field_.empty()) {
    return isOccupied(x, y);
  }
  return distance_field_[Index2Vectornum(x, y)] <= safe_dis;
}

bool CostmapAdapter::isUnOccupied(int x, int y) const
{
  return isWithinBounds(x, y) && !isOccupied(x, y);
}

bool CostmapAdapter::isOccupied(int x, int y) const
{
  if (!isWithinBounds(x, y) || costmap_ == nullptr) {
    return true;
  }
  const unsigned char cost = costmap_->getCost(x, y);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return !unknown_is_free_;
  }
  return cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
}
}  // namespace ddr_navigation
