#include "ddr_icrekf/icrekf.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ddr_navigation::ICREKFNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
