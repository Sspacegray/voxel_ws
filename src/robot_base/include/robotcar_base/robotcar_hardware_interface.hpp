#ifndef ROBOTCAR_HARDWARE_INTERFACE_HPP
#define ROBOTCAR_HARDWARE_INTERFACE_HPP

#include <string>
#include <vector>
#include <atomic>
#include <memory>
#include <thread>
#include <mutex>
#include <libserial/SerialPort.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotcar_base/visibility_control.h"
#include "robotcar_base/msg/car_info.hpp"
#include "robotcar_base/srv/set_control_mode.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace robotcar_base
{

class RobotCarHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RobotCarHardwareInterface)

    ROBOTCAR_BASE_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    ROBOTCAR_BASE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    ROBOTCAR_BASE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    ROBOTCAR_BASE_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    ROBOTCAR_BASE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    ROBOTCAR_BASE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    ROBOTCAR_BASE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Communication with hardware
    LibSerial::SerialPort serial_port_;

    // Node for publishing and services
    rclcpp::Node::SharedPtr non_realtime_node_;
    rclcpp::Publisher<robotcar_base::msg::CarInfo>::SharedPtr car_info_pub_;
    rclcpp::Service<robotcar_base::srv::SetControlMode>::SharedPtr mode_service_;
    
    // --- New Members for Magnetometer ---
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    std::vector<double> hw_magnetic_field_;
    // ------------------------------------

    std::thread node_thread_;

    // Hardware parameters
    std::string serial_port_name_;
    int serial_baud_rate_;

    // Wheel joint states
    double hw_position_left_;
    double hw_velocity_left_;
    double hw_position_right_;
    double hw_velocity_right_;

    // Wheel joint commands
    double hw_command_velocity_left_;
    double hw_command_velocity_right_;

    // Mode commands
    std::atomic<uint16_t> hw_mode1_;
    std::atomic<uint16_t> hw_mode2_;

    // Wheel parameters from URDF
    double wheel_radius_;
    double wheel_separation_;

    // IMU related members
    LibSerial::SerialPort imu_serial_port_;
    std::string imu_port_name_;
    int imu_baud_rate_;
    std::string imu_frame_id_;

    // IMU data storage
    std::vector<double> imu_orientation_;       // Quaternion [x, y, z, w]
    std::vector<double> imu_angular_velocity_; // [x, y, z]
    std::vector<double> imu_linear_acceleration_; // [x, y, z]

    // IMU data reading thread
    std::thread imu_thread_;
    std::atomic<bool> imu_thread_running_{false};
    std::mutex imu_data_mutex_;

    // IMU data parsing methods
    void imuReadThread();
    void processImuPacket(const std::vector<uint8_t>& packet);
    void processHWT901BData(const std::vector<uint8_t>& buffer);
};

}  // namespace robotcar_base

#endif  // ROBOTCAR_HARDWARE_INTERFACE_HPP
