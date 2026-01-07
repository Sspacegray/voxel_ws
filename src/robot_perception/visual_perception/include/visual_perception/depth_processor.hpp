#ifndef VISUAL_PERCEPTION__DEPTH_PROCESSOR_HPP_
#define VISUAL_PERCEPTION__DEPTH_PROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>

namespace visual_perception
{

/**
 * @brief Depth image processor node
 * 
 * This node processes depth images from D435 camera:
 * - Applies filtering (median + bilateral) to reduce noise
 * - Converts depth image to point cloud
 * - Filters points based on height (ground removal)
 * - Publishes obstacle point cloud for costmap integration
 */
class DepthProcessor : public rclcpp::Node
{
public:
  explicit DepthProcessor(const rclcpp::NodeOptions & options);
  ~DepthProcessor() = default;

private:
  // Callbacks
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Processing functions
  cv::Mat filterDepthImage(const cv::Mat & depth);
  pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPointCloud(
    const cv::Mat & depth,
    const image_geometry::PinholeCameraModel & model);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterByHeight(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Camera model
  image_geometry::PinholeCameraModel camera_model_;
  bool camera_info_received_{false};

  // Parameters
  double min_depth_;           // Minimum valid depth (m)
  double max_depth_;           // Maximum valid depth (m)
  double min_height_;          // Minimum height above ground (m)
  double max_height_;          // Maximum height to consider (m)
  double ground_height_;       // Assumed ground height relative to camera (m)
  int filter_kernel_size_;     // Median filter kernel size
  double bilateral_sigma_color_;
  double bilateral_sigma_space_;
  std::string output_frame_;   // Output frame for point cloud
  bool enable_filtering_;
};

}  // namespace visual_perception

#endif  // VISUAL_PERCEPTION__DEPTH_PROCESSOR_HPP_
