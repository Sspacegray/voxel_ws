#include "visual_perception/depth_processor.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <limits>
#include <cmath>

namespace visual_perception
{

DepthProcessor::DepthProcessor(const rclcpp::NodeOptions & options)
: Node("depth_processor", options)
{
  // Declare parameters
  min_depth_ = this->declare_parameter("min_depth", 0.2);
  max_depth_ = this->declare_parameter("max_depth", 5.0);
  min_height_ = this->declare_parameter("min_height", 0.05);
  max_height_ = this->declare_parameter("max_height", 1.5);
  ground_height_ = this->declare_parameter("ground_height", 0.0);
  filter_kernel_size_ = this->declare_parameter("filter_kernel_size", 5);
  bilateral_sigma_color_ = this->declare_parameter("bilateral_sigma_color", 75.0);
  bilateral_sigma_space_ = this->declare_parameter("bilateral_sigma_space", 75.0);
  output_frame_ = this->declare_parameter("output_frame", std::string("base_link"));
  enable_filtering_ = this->declare_parameter("enable_filtering", true);

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/depth/image_rect_raw", rclcpp::SensorDataQoS(),
    std::bind(&DepthProcessor::depthCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera/depth/camera_info", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&DepthProcessor::cameraInfoCallback, this, std::placeholders::_1));

  // Publishers
  obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "depth_obstacle_cloud", rclcpp::SensorDataQoS());

  filtered_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "depth_filtered", rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "DepthProcessor initialized");
  RCLCPP_INFO(this->get_logger(), "  - Depth range: [%.2f, %.2f] m", min_depth_, max_depth_);
  RCLCPP_INFO(this->get_logger(), "  - Height range: [%.2f, %.2f] m", min_height_, max_height_);
  RCLCPP_INFO(this->get_logger(), "  - Output frame: %s", output_frame_.c_str());
}

void DepthProcessor::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!camera_info_received_) {
    camera_model_.fromCameraInfo(msg);
    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera info received: %dx%d",
      msg->width, msg->height);
  }
}

void DepthProcessor::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!camera_info_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Waiting for camera info...");
    return;
  }

  // Convert ROS image to OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception & e) {
    // Try 32FC1 format
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      // Convert float to 16-bit (in mm)
      cv_ptr->image.convertTo(cv_ptr->image, CV_16UC1, 1000.0);
    } catch (cv_bridge::Exception & e2) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e2.what());
      return;
    }
  }

  cv::Mat depth = cv_ptr->image;

  // Apply filtering if enabled
  cv::Mat filtered_depth;
  if (enable_filtering_) {
    filtered_depth = filterDepthImage(depth);
  } else {
    filtered_depth = depth;
  }

  // Publish filtered depth image
  if (filtered_depth_pub_->get_subscription_count() > 0) {
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    out_msg.image = filtered_depth;
    filtered_depth_pub_->publish(*out_msg.toImageMsg());
  }

  // Convert to point cloud
  auto cloud = depthToPointCloud(filtered_depth, camera_model_);

  // Filter by height
  auto obstacle_cloud = filterByHeight(cloud);

  // Publish obstacle cloud
  publishPointCloud(obstacle_cloud);
}

cv::Mat DepthProcessor::filterDepthImage(const cv::Mat & depth)
{
  cv::Mat result;

  // Convert to float for processing
  cv::Mat depth_float;
  depth.convertTo(depth_float, CV_32F);

  // Apply median filter to remove salt-and-pepper noise
  cv::Mat median_filtered;
  cv::medianBlur(depth_float, median_filtered, filter_kernel_size_);

  // Apply bilateral filter to smooth while preserving edges
  cv::Mat bilateral_filtered;
  cv::bilateralFilter(median_filtered, bilateral_filtered,
    -1, bilateral_sigma_color_, bilateral_sigma_space_);

  // Convert back to 16-bit
  bilateral_filtered.convertTo(result, CV_16UC1);

  return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthProcessor::depthToPointCloud(
  const cv::Mat & depth,
  const image_geometry::PinholeCameraModel & model)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->header.frame_id = model.tfFrame();
  cloud->height = 1;
  cloud->is_dense = false;

  // Camera intrinsics
  double fx = model.fx();
  double fy = model.fy();
  double cx = model.cx();
  double cy = model.cy();

  // Reserve space (rough estimate)
  cloud->points.reserve(depth.rows * depth.cols / 4);

  // Convert depth image to 3D points
  for (int v = 0; v < depth.rows; v += 2) {  // Skip every other row for performance
    for (int u = 0; u < depth.cols; u += 2) {  // Skip every other column
      uint16_t d = depth.at<uint16_t>(v, u);
      
      if (d == 0) continue;

      double z = static_cast<double>(d) / 1000.0;  // Convert mm to m
      
      if (z < min_depth_ || z > max_depth_) continue;

      double x = (u - cx) * z / fx;
      double y = (v - cy) * z / fy;

      pcl::PointXYZ point;
      point.x = static_cast<float>(z);   // Forward
      point.y = static_cast<float>(-x);  // Left
      point.z = static_cast<float>(-y);  // Up
      cloud->points.push_back(point);
    }
  }

  cloud->width = cloud->points.size();
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthProcessor::filterByHeight(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filtered->header = cloud->header;
  filtered->height = 1;
  filtered->is_dense = false;

  filtered->points.reserve(cloud->points.size());

  for (const auto & point : cloud->points) {
    // z in camera frame corresponds to height
    // Filter points within height range (above ground, below max height)
    if (point.z >= min_height_ && point.z <= max_height_) {
      filtered->points.push_back(point);
    }
  }

  filtered->width = filtered->points.size();
  return filtered;
}

void DepthProcessor::publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  if (obstacle_cloud_pub_->get_subscription_count() == 0) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp = this->now();
  cloud_msg.header.frame_id = output_frame_;

  // Try to transform to output frame if different from camera frame
  if (cloud->header.frame_id != output_frame_) {
    try {
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_->lookupTransform(
        output_frame_, cloud->header.frame_id,
        tf2::TimePointZero);
      
      sensor_msgs::msg::PointCloud2 transformed_cloud;
      tf2::doTransform(cloud_msg, transformed_cloud, transform);
      obstacle_cloud_pub_->publish(transformed_cloud);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Could not transform point cloud: %s", ex.what());
      // Publish without transformation
      cloud_msg.header.frame_id = cloud->header.frame_id;
      obstacle_cloud_pub_->publish(cloud_msg);
    }
  } else {
    obstacle_cloud_pub_->publish(cloud_msg);
  }
}

}  // namespace visual_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(visual_perception::DepthProcessor)
