#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <Eigen/Dense>
#include <cmath>

class TiltCompensator : public rclcpp::Node
{
public:
    TiltCompensator() : Node("tilt_compensator")
    {
        // Parameters
        this->declare_parameter("pitch_deg", 45.0);
        double pitch_deg = this->get_parameter("pitch_deg").as_double();
        
        // Convert to radians
        double pitch_rad = pitch_deg * M_PI / 180.0;
        
        // Create rotation matrix (Rotation around Y axis)
        // If sensor is tilted down by 45 deg (pitch = -45 in ROS frame),
        // we need to rotate data by +45 deg around Y to make it horizontal.
        // R_y(theta) = [ cos  0  sin]
        //              [  0   1   0 ]
        //              [-sin  0  cos]
        
        rot_matrix_ = Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
        
        RCLCPP_INFO(this->get_logger(), "Tilt Compensator started with pitch: %.2f degrees", pitch_deg);
        
        // Subscribers
        sub_lidar_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10, std::bind(&TiltCompensator::lidar_callback, this, std::placeholders::_1));
            
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, std::bind(&TiltCompensator::imu_callback, this, std::placeholders::_1));
            
        // Publishers
        pub_lidar_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/lidar/cloud", 10);
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/lidar/imu", 10);
    }

private:
    void lidar_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        auto new_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
        *new_msg = *msg; // Copy header and other fields
        
        // Force synchronization with ROS time to prevent timestamp issues
        // complex offset logic causing 150s+ lags or future timestamps.
        // Simple approach: Use current ROS time.
        rclcpp::Time corrected_time = this->now();
        new_msg->header.stamp = corrected_time;
        
        // Reserve memory
        new_msg->points.reserve(msg->points.size());
        
        // Process points
        for (auto & point : new_msg->points) {
            Eigen::Vector3d p(point.x, point.y, point.z);
            Eigen::Vector3d p_rot = rot_matrix_ * p;
            
            point.x = p_rot.x();
            point.y = p_rot.y();
            point.z = p_rot.z();
        }
        
        pub_lidar_->publish(std::move(new_msg));
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto new_msg = std::make_unique<sensor_msgs::msg::Imu>();
        *new_msg = *msg;
        
        // Force sync with ROS time
        new_msg->header.stamp = this->now();
        
        // Rotate linear acceleration
        Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Eigen::Vector3d acc_rot = rot_matrix_ * acc;
        new_msg->linear_acceleration.x = acc_rot.x();
        new_msg->linear_acceleration.y = acc_rot.y();
        new_msg->linear_acceleration.z = acc_rot.z();
        
        // Rotate angular velocity
        Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        Eigen::Vector3d gyro_rot = rot_matrix_ * gyro;
        new_msg->angular_velocity.x = gyro_rot.x();
        new_msg->angular_velocity.y = gyro_rot.y();
        new_msg->angular_velocity.z = gyro_rot.z();
        
        // Rotate orientation (Quaternion)
        Eigen::Quaterniond q_orig(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        Eigen::Quaterniond q_rot(rot_matrix_);
        Eigen::Quaterniond q_new = q_rot * q_orig;
        
        new_msg->orientation.w = q_new.w();
        new_msg->orientation.x = q_new.x();
        new_msg->orientation.y = q_new.y();
        new_msg->orientation.z = q_new.z();
        
        pub_imu_->publish(std::move(new_msg));
    }

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    
    Eigen::Matrix3d rot_matrix_;
    double time_offset_ = 0.0;
    bool offset_initialized_ = false;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TiltCompensator>());
    rclcpp::shutdown();
    return 0;
}
