#include "plan_env/sdf_map.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("esdf_server");
    
    RCLCPP_INFO(node->get_logger(), "=== ESDF Server Node ===");
    RCLCPP_INFO(node->get_logger(), "订阅栅格地图，生成并发布ESDF地图");
    RCLCPP_INFO(node->get_logger(), "可视化话题:");
    RCLCPP_INFO(node->get_logger(), "  - /sdf_map/occupancy (PointCloud2): 障碍物点云");
    RCLCPP_INFO(node->get_logger(), "  - /sdf_map/esdf (PointCloud2): ESDF距离场可视化");
    
    SDFMap sdf_map;
    sdf_map.initMap(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
