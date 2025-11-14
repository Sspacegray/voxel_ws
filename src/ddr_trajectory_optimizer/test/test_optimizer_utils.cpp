#include <gtest/gtest.h>

#include <vector>

#include "ddr_trajectory_optimizer/ddr_trajectory_optimizer.hpp"

class TestableOptimizer : public ddr_navigation::DdrTrajectoryOptimizer
{
public:
  using Path2D = ddr_navigation::DdrTrajectoryOptimizer::Path2D;
  using ddr_navigation::DdrTrajectoryOptimizer::computeCurvature;
  using ddr_navigation::DdrTrajectoryOptimizer::resamplePath;
};

TEST(TrajectoryOptimizerTest, ResampleImprovesResolution)
{
  TestableOptimizer optimizer;
  TestableOptimizer::Path2D path;
  path.emplace_back(0.0, 0.0);
  path.emplace_back(0.3, 0.0);
  const auto resampled = optimizer.resamplePath(path);
  EXPECT_GT(resampled.size(), path.size());
  EXPECT_DOUBLE_EQ(resampled.front().x(), 0.0);
  EXPECT_DOUBLE_EQ(resampled.back().x(), 0.3);
}

TEST(TrajectoryOptimizerTest, CurvatureZeroForStraightLine)
{
  TestableOptimizer optimizer;
  const Eigen::Vector2d p0(0.0, 0.0);
  const Eigen::Vector2d p1(0.5, 0.0);
  const Eigen::Vector2d p2(1.0, 0.0);
  EXPECT_NEAR(optimizer.computeCurvature(p0, p1, p2), 0.0, 1e-6);
}
