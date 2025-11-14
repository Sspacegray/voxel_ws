#include <gtest/gtest.h>

#include "ddr_mpc_controller/ddr_mpc_controller.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class TestableMpc : public ddr_navigation::DdrMpcController
{
public:
  double callNormalizeAngle(double angle) const { return normalizeAngle(angle); }
  double callClamp(double value, double low, double high) const { return clamp(value, low, high); }
  ReferenceSignals callBuildReference(
    double arc, double yaw, const Eigen::Vector2d & pos) const
  {
    return buildReferenceSignals(arc, yaw, pos);
  }
};

static nav_msgs::msg::Path makeStraightPath(double length, double step)
{
  nav_msgs::msg::Path path;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  for (double x = 0.0; x <= length + 1e-6; x += step) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  }
  return path;
}

TEST(DdrMpcControllerTest, NormalizeAngleWrapsIntoInterval)
{
  TestableMpc controller;
  EXPECT_NEAR(controller.callNormalizeAngle(4.5), -1.783185, 1e-6);
  EXPECT_NEAR(controller.callNormalizeAngle(-4.0), 2.283185, 1e-6);
}

TEST(DdrMpcControllerTest, ClampRespectsBounds)
{
  TestableMpc controller;
  EXPECT_DOUBLE_EQ(controller.callClamp(5.0, -1.0, 2.0), 2.0);
  EXPECT_DOUBLE_EQ(controller.callClamp(-5.0, -1.0, 2.0), -1.0);
  EXPECT_DOUBLE_EQ(controller.callClamp(0.5, -1.0, 2.0), 0.5);
}

TEST(DdrMpcControllerTest, ReferenceSignalsFollowPlan)
{
  TestableMpc controller;
  controller.setPlan(makeStraightPath(1.0, 0.1));
  const auto refs = controller.callBuildReference(0.0, 0.0, Eigen::Vector2d::Zero());
  ASSERT_FALSE(refs.linear.empty());
  ASSERT_FALSE(refs.angular.empty());
  EXPECT_GT(refs.linear.front(), 0.0);
  EXPECT_NEAR(refs.angular.front(), 0.0, 1e-6);
}
