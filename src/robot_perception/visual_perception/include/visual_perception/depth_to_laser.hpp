#ifndef VISUAL_PERCEPTION__DEPTH_TO_LASER_HPP_
#define VISUAL_PERCEPTION__DEPTH_TO_LASER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>

namespace visual_perception
{

/**
 * @brief Converts depth image to virtual laser scan
 * 
 * This node converts D435 depth images to LaserScan messages:
 * - Projects depth image into virtual 2D laser scan
 * - Configurable height range for obstacle detection
 * - Output compatible with Nav2 costmap layers
 * 
 * Algorithm:
 * 1. For each column in the depth image, find minimum depth within height range
 * 2. Use camera intrinsics to convert pixel column to angle
 * 3. Generate LaserScan message with ranges corresponding to each angle
 */
class DepthToLaser : public rclcpp::Node
{
public:
  explicit DepthToLaser(const rclcpp::NodeOptions & options);
  ~DepthToLaser() = default;

private:
  // Callbacks
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Processing functions
  sensor_msgs::msg::LaserScan::UniquePtr convertToLaserScan(
    const cv::Mat & depth,
    const std_msgs::msg::Header & header);
  
  // Helper to compute row range for height filtering
  std::pair<int, int> computeRowRange(double height_min, double height_max);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Camera model
  image_geometry::PinholeCameraModel camera_model_;
  bool camera_info_received_{false};

  // Precomputed values (computed once when camera_info received)
  std::vector<double> col_to_angle_;  // Mapping from column index to angle
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  int scan_height_min_row_;
  int scan_height_max_row_;

  // Parameters
  double min_depth_;           // Minimum valid depth (m)
  double max_depth_;           // Maximum valid depth (m)  
  double scan_height_min_;     // Min height relative to camera optical center (m)
  double scan_height_max_;     // Max height relative to camera optical center (m)
  std::string output_frame_;   // Frame ID for laser scan
  double scan_time_;           // Time between scans (for timing info)
  bool use_inf_;               // Use infinity for max range or max_depth
  int downsample_factor_;      // Column downsampling factor for performance
};

}  // namespace visual_perception

#endif  // VISUAL_PERCEPTION__DEPTH_TO_LASER_HPP_
