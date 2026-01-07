#include "visual_perception/road_segmentation.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

#include <algorithm>
#include <cmath>

namespace visual_perception
{

RoadSegmentation::RoadSegmentation(const rclcpp::NodeOptions & options)
: Node("road_segmentation", options)
{
  // Declare parameters
  min_depth_ = this->declare_parameter("min_depth", 0.2);
  max_depth_ = this->declare_parameter("max_depth", 5.0);
  plane_distance_threshold_ = this->declare_parameter("plane_distance_threshold", 0.03);
  min_plane_points_ = this->declare_parameter("min_plane_points", 500);
  max_slope_ = this->declare_parameter("max_slope", 0.3);  // tan(~17 degrees)
  ransac_max_iterations_ = this->declare_parameter("ransac_max_iterations", 100);
  use_color_ = this->declare_parameter("use_color", false);
  downsampling_resolution_ = this->declare_parameter("downsampling_resolution", 0.02);
  output_frame_ = this->declare_parameter("output_frame", std::string("camera_link"));

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  traversable_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "traversable_mask", rclcpp::SensorDataQoS());
  ground_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "ground_cloud", rclcpp::SensorDataQoS());
  obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "visual_obstacle_cloud", rclcpp::SensorDataQoS());

  // Camera info subscriber
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera/depth/camera_info", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&RoadSegmentation::cameraInfoCallback, this, std::placeholders::_1));

  if (use_color_) {
    // Use synchronized depth + color
    depth_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "camera/depth/image_rect_raw", rmw_qos_profile_sensor_data);
    color_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "camera/color/image_raw", rmw_qos_profile_sensor_data);
    
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *depth_filter_sub_, *color_filter_sub_);
    sync_->registerCallback(
      std::bind(&RoadSegmentation::syncCallback, this, 
        std::placeholders::_1, std::placeholders::_2));
  } else {
    // Depth only mode
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/depth/image_rect_raw", rclcpp::SensorDataQoS(),
      std::bind(&RoadSegmentation::depthCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(this->get_logger(), "RoadSegmentation initialized");
  RCLCPP_INFO(this->get_logger(), "  - Depth range: [%.2f, %.2f] m", min_depth_, max_depth_);
  RCLCPP_INFO(this->get_logger(), "  - Plane distance threshold: %.3f m", plane_distance_threshold_);
  RCLCPP_INFO(this->get_logger(), "  - Max slope: %.2f (%.1f deg)", 
    max_slope_, std::atan(max_slope_) * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "  - Use color: %s", use_color_ ? "true" : "false");
}

void RoadSegmentation::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!camera_info_received_) {
    camera_model_.fromCameraInfo(msg);
    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera info received: %dx%d",
      msg->width, msg->height);
  }
}

void RoadSegmentation::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!camera_info_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Waiting for camera info...");
    return;
  }

  // Convert ROS image to OpenCV
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat depth;
  if (cv_ptr->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    depth = cv_ptr->image;
  } else if (cv_ptr->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    cv_ptr->image.convertTo(depth, CV_16UC1, 1000.0);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unsupported depth encoding: %s", cv_ptr->encoding.c_str());
    return;
  }

  // Convert depth to point cloud
  auto cloud = depthToPointCloud(depth);

  // Segment ground plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector4f plane_coefficients;

  bool ground_found = segmentGround(cloud, ground_cloud, obstacle_cloud, plane_coefficients);

  // Create traversable mask
  cv::Mat mask;
  if (ground_found) {
    mask = createTraversableMask(depth, ground_cloud, plane_coefficients);
  } else {
    // No ground found - entire image is obstacle/unknown
    mask = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC1);
  }

  // Publish results
  publishResults(mask, ground_cloud, obstacle_cloud, msg->header);
}

void RoadSegmentation::syncCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & /*color_msg*/)
{
  // For now, we just process depth
  // Color could be used for enhanced segmentation in the future
  auto depth_copy = std::make_shared<sensor_msgs::msg::Image>(*depth_msg);
  depthCallback(depth_copy);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RoadSegmentation::depthToPointCloud(const cv::Mat & depth)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->header.frame_id = camera_model_.tfFrame();
  cloud->height = 1;
  cloud->is_dense = false;

  double fx = camera_model_.fx();
  double fy = camera_model_.fy();
  double cx = camera_model_.cx();
  double cy = camera_model_.cy();

  // Reserve space
  cloud->points.reserve(depth.rows * depth.cols / 4);

  // Downsample during conversion
  int step = std::max(1, static_cast<int>(downsampling_resolution_ * 50));

  for (int v = 0; v < depth.rows; v += step) {
    for (int u = 0; u < depth.cols; u += step) {
      uint16_t d = depth.at<uint16_t>(v, u);
      if (d == 0) continue;

      double z = static_cast<double>(d) / 1000.0;
      if (z < min_depth_ || z > max_depth_) continue;

      double x = (u - cx) * z / fx;
      double y = (v - cy) * z / fy;

      pcl::PointXYZ point;
      // Convert from optical frame to camera frame convention
      // optical: x-right, y-down, z-forward
      // camera: x-forward, y-left, z-up
      point.x = static_cast<float>(z);    // forward
      point.y = static_cast<float>(-x);   // left
      point.z = static_cast<float>(-y);   // up
      cloud->points.push_back(point);
    }
  }

  cloud->width = cloud->points.size();
  return cloud;
}

bool RoadSegmentation::segmentGround(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & ground_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & obstacle_cloud,
  Eigen::Vector4f & plane_coefficients)
{
  if (cloud->points.size() < static_cast<size_t>(min_plane_points_)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Not enough points for ground segmentation: %zu", cloud->points.size());
    return false;
  }

  // RANSAC plane segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(ransac_max_iterations_);
  seg.setDistanceThreshold(plane_distance_threshold_);
  
  // Ground plane should be approximately perpendicular to Z axis (vertical)
  // In camera frame: z is up, so ground plane normal should be close to (0, 0, 1)
  Eigen::Vector3f axis(0.0f, 0.0f, 1.0f);
  seg.setAxis(axis);
  seg.setEpsAngle(std::atan(max_slope_));  // Allow some slope

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() < static_cast<size_t>(min_plane_points_)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Ground plane not found - only %zu inliers", inliers->indices.size());
    return false;
  }

  // Store plane coefficients
  plane_coefficients[0] = coefficients->values[0];
  plane_coefficients[1] = coefficients->values[1];
  plane_coefficients[2] = coefficients->values[2];
  plane_coefficients[3] = coefficients->values[3];

  // Extract ground and obstacle points
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  extract.setNegative(false);
  extract.filter(*ground_cloud);

  extract.setNegative(true);
  extract.filter(*obstacle_cloud);

  return true;
}

cv::Mat RoadSegmentation::createTraversableMask(
  const cv::Mat & depth,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & ground_cloud,
  const Eigen::Vector4f & plane_coefficients)
{
  cv::Mat mask = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC1);

  double fx = camera_model_.fx();
  double fy = camera_model_.fy();
  double cx = camera_model_.cx();
  double cy = camera_model_.cy();

  float a = plane_coefficients[0];
  float b = plane_coefficients[1];
  float c = plane_coefficients[2];
  float d = plane_coefficients[3];

  // For each pixel, check if the 3D point lies on the ground plane
  for (int v = 0; v < depth.rows; ++v) {
    for (int u = 0; u < depth.cols; ++u) {
      uint16_t depth_val = depth.at<uint16_t>(v, u);
      if (depth_val == 0) continue;

      double z = static_cast<double>(depth_val) / 1000.0;
      if (z < min_depth_ || z > max_depth_) continue;

      double x_optical = (u - cx) * z / fx;
      double y_optical = (v - cy) * z / fy;

      // Convert to camera frame
      float x = static_cast<float>(z);
      float y = static_cast<float>(-x_optical);
      float z_pt = static_cast<float>(-y_optical);

      // Check distance to plane
      float distance = std::abs(a * x + b * y + c * z_pt + d);

      if (distance < plane_distance_threshold_ * 2.0f) {
        mask.at<uchar>(v, u) = 255;  // Traversable
      }
    }
  }

  // Apply morphological operations to clean up the mask
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  return mask;
}

void RoadSegmentation::publishResults(
  const cv::Mat & mask,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & ground_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & obstacle_cloud,
  const std_msgs::msg::Header & header)
{
  // Publish traversable mask
  if (traversable_mask_pub_->get_subscription_count() > 0) {
    cv_bridge::CvImage mask_msg;
    mask_msg.header = header;
    mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
    mask_msg.image = mask;
    traversable_mask_pub_->publish(*mask_msg.toImageMsg());
  }

  // Publish ground cloud
  if (ground_cloud_pub_->get_subscription_count() > 0 && !ground_cloud->points.empty()) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*ground_cloud, cloud_msg);
    cloud_msg.header = header;
    cloud_msg.header.frame_id = output_frame_;
    ground_cloud_pub_->publish(cloud_msg);
  }

  // Publish obstacle cloud
  if (obstacle_cloud_pub_->get_subscription_count() > 0 && !obstacle_cloud->points.empty()) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*obstacle_cloud, cloud_msg);
    cloud_msg.header = header;
    cloud_msg.header.frame_id = output_frame_;
    obstacle_cloud_pub_->publish(cloud_msg);
  }
}

}  // namespace visual_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(visual_perception::RoadSegmentation)
