#include "visual_perception/road_segmentation.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<visual_perception::RoadSegmentation>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
