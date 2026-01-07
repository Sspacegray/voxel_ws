#include "visual_perception/depth_to_laser.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace visual_perception
{

DepthToLaser::DepthToLaser(const rclcpp::NodeOptions & options)
: Node("depth_to_laser", options)
{
  // Declare parameters
  min_depth_ = this->declare_parameter("min_depth", 0.2);
  max_depth_ = this->declare_parameter("max_depth", 5.0);
  scan_height_min_ = this->declare_parameter("scan_height_min", -0.3);
  scan_height_max_ = this->declare_parameter("scan_height_max", 0.5);
  output_frame_ = this->declare_parameter("output_frame", std::string("camera_depth_optical_frame"));
  scan_time_ = this->declare_parameter("scan_time", 0.033);
  use_inf_ = this->declare_parameter("use_inf", true);
  downsample_factor_ = this->declare_parameter("downsample_factor", 2);

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribers
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/depth/image_rect_raw", rclcpp::SensorDataQoS(),
    std::bind(&DepthToLaser::depthCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera/depth/camera_info", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&DepthToLaser::cameraInfoCallback, this, std::placeholders::_1));

  // Publishers
  laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "visual_scan", rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "DepthToLaser initialized");
  RCLCPP_INFO(this->get_logger(), "  - Depth range: [%.2f, %.2f] m", min_depth_, max_depth_);
  RCLCPP_INFO(this->get_logger(), "  - Scan height range: [%.2f, %.2f] m", 
    scan_height_min_, scan_height_max_);
  RCLCPP_INFO(this->get_logger(), "  - Output frame: %s", output_frame_.c_str());
}

void DepthToLaser::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!camera_info_received_) {
    camera_model_.fromCameraInfo(msg);

    int width = msg->width;
    int height = msg->height;

    // Compute column to angle mapping
    col_to_angle_.resize(width);
    double fx = camera_model_.fx();
    double cx = camera_model_.cx();

    for (int u = 0; u < width; ++u) {
      // Angle from optical axis
      col_to_angle_[u] = std::atan2(static_cast<double>(u) - cx, fx);
    }

    // Compute angle range (D435 has ~87° horizontal FOV)
    angle_min_ = col_to_angle_.front();
    angle_max_ = col_to_angle_.back();
    angle_increment_ = (angle_max_ - angle_min_) / (width - 1);

    // Compute row range for height filtering
    double fy = camera_model_.fy();
    double cy = camera_model_.cy();

    // For a point at distance z, y = (v - cy) * z / fy
    // We want to find rows that correspond to scan_height_min/max at typical distances
    // Using a reference depth of 1m for row calculation
    double ref_depth = 1.0;
    
    // In optical frame: +y is down, +z is forward
    // scan_height_min is the lower part of the image (higher row numbers)
    // scan_height_max is the upper part of the image (lower row numbers)
    
    int row_for_min_height = static_cast<int>(cy + scan_height_max_ * fy / ref_depth);
    int row_for_max_height = static_cast<int>(cy + scan_height_min_ * fy / ref_depth);

    scan_height_min_row_ = std::max(0, std::min(row_for_max_height, row_for_min_height));
    scan_height_max_row_ = std::min(height - 1, std::max(row_for_max_height, row_for_min_height));

    RCLCPP_INFO(this->get_logger(), "Camera info received: %dx%d, fx=%.2f, fy=%.2f",
      width, height, fx, fy);
    RCLCPP_INFO(this->get_logger(), "  - Angle range: [%.2f, %.2f] rad (%.1f, %.1f deg)",
      angle_min_, angle_max_, 
      angle_min_ * 180.0 / M_PI, angle_max_ * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  - Scan rows: [%d, %d]",
      scan_height_min_row_, scan_height_max_row_);

    camera_info_received_ = true;
  }
}

void DepthToLaser::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!camera_info_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Waiting for camera info...");
    return;
  }

  if (laser_pub_->get_subscription_count() == 0) {
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
    // Convert 32FC1 (meters) to 16UC1 (millimeters)
    cv_ptr->image.convertTo(depth, CV_16UC1, 1000.0);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unsupported depth encoding: %s", cv_ptr->encoding.c_str());
    return;
  }

  // Convert to laser scan
  auto scan = convertToLaserScan(depth, msg->header);
  if (scan) {
    laser_pub_->publish(std::move(scan));
  }
}

sensor_msgs::msg::LaserScan::UniquePtr DepthToLaser::convertToLaserScan(
  const cv::Mat & depth,
  const std_msgs::msg::Header & header)
{
  auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();

  scan->header = header;
  scan->header.frame_id = output_frame_;
  
  // Calculate number of ranges based on downsampling
  int num_ranges = depth.cols / downsample_factor_;
  
  scan->angle_min = angle_min_;
  scan->angle_max = angle_max_;
  scan->angle_increment = (angle_max_ - angle_min_) / (num_ranges - 1);
  scan->time_increment = scan_time_ / num_ranges;
  scan->scan_time = scan_time_;
  scan->range_min = static_cast<float>(min_depth_);
  scan->range_max = static_cast<float>(max_depth_);

  scan->ranges.resize(num_ranges, std::numeric_limits<float>::infinity());

  // Ensure row range is valid
  int row_start = std::max(0, scan_height_min_row_);
  int row_end = std::min(depth.rows - 1, scan_height_max_row_);

  // For each column, find minimum depth in the height range
  for (int col = 0; col < depth.cols; col += downsample_factor_) {
    int scan_idx = col / downsample_factor_;
    float min_range = std::numeric_limits<float>::infinity();

    for (int row = row_start; row <= row_end; ++row) {
      uint16_t d = depth.at<uint16_t>(row, col);
      if (d == 0) continue;

      float range = static_cast<float>(d) / 1000.0f;  // mm to m

      if (range >= min_depth_ && range <= max_depth_) {
        min_range = std::min(min_range, range);
      }
    }

    // Note: D435 image is ordered left-to-right which corresponds to
    // positive to negative angles in optical frame convention
    // We reverse the order to match standard laser scan convention
    int reversed_idx = num_ranges - 1 - scan_idx;
    if (reversed_idx >= 0 && reversed_idx < num_ranges) {
      if (use_inf_ && std::isinf(min_range)) {
        scan->ranges[reversed_idx] = std::numeric_limits<float>::infinity();
      } else {
        scan->ranges[reversed_idx] = min_range;
      }
    }
  }

  return scan;
}

std::pair<int, int> DepthToLaser::computeRowRange(double height_min, double height_max)
{
  // This function is used to dynamically compute row range based on depth
  // Currently using static computation in cameraInfoCallback
  return std::make_pair(scan_height_min_row_, scan_height_max_row_);
}

}  // namespace visual_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(visual_perception::DepthToLaser)
