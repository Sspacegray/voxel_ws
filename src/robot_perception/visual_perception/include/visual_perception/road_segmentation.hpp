#ifndef VISUAL_PERCEPTION__ROAD_SEGMENTATION_HPP_
#define VISUAL_PERCEPTION__ROAD_SEGMENTATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>

namespace visual_perception
{

/**
 * @brief Road/Traversable area segmentation node
 * 
 * This node segments traversable road areas from depth images:
 * - Uses RANSAC to fit ground plane from depth data
 * - Identifies areas that are flat and at ground level
 * - Publishes traversable mask for visualization and planning
 * - Publishes ground point cloud for debugging
 * 
 * Algorithm:
 * 1. Convert depth image to point cloud
 * 2. Apply RANSAC plane fitting to find dominant ground plane
 * 3. Classify points as ground/non-ground based on distance to plane
 * 4. Project ground classification back to image as traversable mask
 */
class RoadSegmentation : public rclcpp::Node
{
public:
  explicit RoadSegmentation(const rclcpp::NodeOptions & options);
  ~RoadSegmentation() = default;

private:
  // Callback for synchronized depth + color 
  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg);
  
  // Callback for depth-only mode
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Processing functions
  pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPointCloud(const cv::Mat & depth);
  
  /**
   * @brief Segment ground plane using RANSAC
   * @param cloud Input point cloud
   * @param ground_cloud Output: points on ground plane
   * @param obstacle_cloud Output: points not on ground (obstacles)
   * @param plane_coefficients Output: plane model [a, b, c, d] where ax+by+cz+d=0
   * @return true if ground plane found
   */
  bool segmentGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & ground_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & obstacle_cloud,
    Eigen::Vector4f & plane_coefficients);
  
  /**
   * @brief Create traversable mask from ground segmentation
   * @param depth Original depth image
   * @param ground_cloud Points classified as ground
   * @return Binary mask where white (255) = traversable, black (0) = obstacle/unknown
   */
  cv::Mat createTraversableMask(
    const cv::Mat & depth,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & ground_cloud,
    const Eigen::Vector4f & plane_coefficients);

  void publishResults(
    const cv::Mat & mask,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & ground_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & obstacle_cloud,
    const std_msgs::msg::Header & header);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  
  // Message filter subscribers for synchronized depth+color
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_filter_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> color_filter_sub_;
  
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr traversable_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Camera model
  image_geometry::PinholeCameraModel camera_model_;
  bool camera_info_received_{false};

  // Parameters
  double min_depth_;                    // Minimum valid depth (m)
  double max_depth_;                    // Maximum valid depth (m)
  double plane_distance_threshold_;     // RANSAC inlier distance threshold
  int min_plane_points_;                // Minimum points for valid plane
  double max_slope_;                    // Maximum ground slope (tan value)
  int ransac_max_iterations_;           // RANSAC iterations
  bool use_color_;                      // Whether to use color image
  double downsampling_resolution_;      // Voxel grid downsampling resolution
  std::string output_frame_;            // Output frame for visualization
};

}  // namespace visual_perception

#endif  // VISUAL_PERCEPTION__ROAD_SEGMENTATION_HPP_
