#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "ddr_jps_global_planner/costmap_adapter.hpp"
#include "ddr_jps_global_planner/graph_search.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace
{
struct AdapterBundle
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  std::shared_ptr<ddr_navigation::CostmapAdapter> adapter;
};

AdapterBundle makeAdapter(unsigned int size_x, unsigned int size_y)
{
  AdapterBundle bundle;
  bundle.costmap = std::shared_ptr<nav2_costmap_2d::Costmap2D>(
    new nav2_costmap_2d::Costmap2D(size_x, size_y, 0.1, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE));
  bundle.adapter = std::make_shared<ddr_navigation::CostmapAdapter>(bundle.costmap.get(), false);
  bundle.adapter->updateDistanceField();
  return bundle;
}
}  // namespace

TEST(GraphSearchTest, FindsPathInFreeSpace)
{
  auto bundle = makeAdapter(10, 10);
  JPS::GraphSearch search(bundle.adapter, 0.0);
  EXPECT_TRUE(search.plan(0, 0, 9, 9, true, -1));
  const auto path = search.getPath();
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front()->x, 9);
  EXPECT_EQ(path.back()->x, 0);
}

TEST(GraphSearchTest, RespectsObstacles)
{
  auto bundle = makeAdapter(5, 5);
  bundle.costmap->setCost(2, 2, nav2_costmap_2d::LETHAL_OBSTACLE);
  bundle.adapter->updateDistanceField();
  JPS::GraphSearch search(bundle.adapter, 0.1);
  EXPECT_TRUE(search.plan(0, 0, 4, 4, true, -1));
  for (const auto & state : search.getPath()) {
    EXPECT_FALSE(state->x == 2 && state->y == 2);
  }
}
