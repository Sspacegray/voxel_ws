#include "robotcar_base/robotcar_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <stdexcept>
#include <numeric>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace robotcar_base
{

// Helper struct for serial communication
#pragma pack (1)
typedef struct _REPORT_DATA_ 
{
    unsigned char Head_1;   
    unsigned char Head_2; 
    unsigned char cmd_1; 
    unsigned char flag_1;
    unsigned char flag_2;
    int Speed_X;    
    int Speed_Y;          
    int Speed_Z;          
    float power;      
    unsigned char  Sum;           
}REPORT_DATA;

typedef struct _CMD_DATA_ 
{
    unsigned char Head_1;   
    unsigned char Head_2; 
    unsigned char cmd_1; 
    unsigned char cmd_2;
    unsigned short mode1;
    unsigned short mode2;
    unsigned int  Speed_X;      
    unsigned int  Speed_Y;  
    unsigned int  Speed_Z;        
    unsigned char  Sum;             
}CMD_DATA;
#pragma pack ()

uint8_t CalcChecksum(uint8_t *pBuf, uint32_t nLength)
{
    uint8_t checksum = 0;
    for(uint32_t i = 0; i < (nLength-1); i++)
    {
        checksum ^= pBuf[i];
    }
    return checksum;
}

// Helper function: Check IMU packet checksum
bool imuChecksumOk(const std::vector<uint8_t>& packet)
{
    if (packet.size() != 11) return false;
    uint8_t calculated_checksum = 0;
    for (size_t i = 0; i < 10; ++i) {
        calculated_checksum += packet[i];
    }
    return calculated_checksum == packet[10];
}

// Helper function: Calculate quaternion from Euler angles (radians)
std::vector<double> quaternionFromEuler(double roll, double pitch, double yaw)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    std::vector<double> q(4);
    q[3] = cr * cp * cy + sr * sp * sy; // w
    q[0] = sr * cp * cy - cr * sp * sy; // x
    q[1] = cr * sp * cy + sr * cp * sy; // y
    q[2] = cr * cp * sy - sr * sp * cy; // z
    return q;
}


hardware_interface::CallbackReturn RobotCarHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Get hardware parameters
    serial_port_name_ = info_.hardware_parameters["serial_port_name"];
    serial_baud_rate_ = std::stoi(info_.hardware_parameters["serial_baud_rate"]);
    wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
    wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);

    // Get IMU parameters (Optional)
    if (info_.hardware_parameters.count("imu_port")) {
        imu_port_name_ = info_.hardware_parameters.at("imu_port");
        imu_baud_rate_ = std::stoi(info_.hardware_parameters.at("imu_baud_rate"));
        imu_frame_id_ = info_.hardware_parameters.at("imu_frame_id");
        use_imu_ = true;
    } else {
        use_imu_ = false;
        RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "IMU parameters not found, disabling internal IMU.");
    }

    // Initialize states and commands
    hw_position_left_ = 0.0;
    hw_velocity_left_ = 0.0;
    hw_position_right_ = 0.0;
    hw_velocity_right_ = 0.0;
    hw_command_velocity_left_ = 0.0;
    hw_command_velocity_right_ = 0.0;
    hw_mode1_.store(0);
    hw_mode2_.store(0);

    // Initialize IMU data
    imu_orientation_.resize(4, 0.0);      // [x, y, z, w]
    imu_orientation_[3] = 1.0;            // w = 1 (unit quaternion)
    imu_angular_velocity_.resize(3, 0.0); // [x, y, z]
    imu_linear_acceleration_.resize(3, 0.0); // [x, y, z]
    hw_magnetic_field_.resize(3, 0.0);    // Magnetometer data [x, y, z]

    // Create a node for non-real-time communication
    non_realtime_node_ = rclcpp::Node::make_shared(info_.name + "_non_realtime_node");
    car_info_pub_ = non_realtime_node_->create_publisher<robot_base::msg::CarInfo>("/car_info", rclcpp::SystemDefaultsQoS());
    mag_pub_ = non_realtime_node_->create_publisher<sensor_msgs::msg::MagneticField>("/wit/mag", rclcpp::SystemDefaultsQoS());

    // Create the service server for setting control modes
    auto service_callback =
      [this](const std::shared_ptr<robot_base::srv::SetControlMode::Request> request,
             std::shared_ptr<robot_base::srv::SetControlMode::Response>      response)
      {
        RCLCPP_INFO(
          rclcpp::get_logger("RobotCarHardwareInterface"),
          "Setting control mode: mode1=%d, mode2=%d", request->mode1, request->mode2);
        this->hw_mode1_.store(request->mode1);
        this->hw_mode2_.store(request->mode2);
        response->success = true;
      };
    mode_service_ = non_realtime_node_->create_service<robot_base::srv::SetControlMode>("set_control_mode", service_callback);

    node_thread_ = std::thread([this]() { rclcpp::spin(this->non_realtime_node_); });

    RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "on_init successful");
    return hardware_interface::CallbackReturn::SUCCESS;
}

RobotCarHardwareInterface::~RobotCarHardwareInterface()
{
    // Stop IMU thread
    if (imu_thread_running_.load())
    {
        imu_thread_running_.store(false);
        if (imu_thread_.joinable())
        {
            imu_thread_.join();
        }
    }

    // Close serial ports
    if (serial_port_.IsOpen())
    {
        serial_port_.Close();
    }
    if (use_imu_ && imu_serial_port_.IsOpen())
    {
        imu_serial_port_.Close();
    }
    if (node_thread_.joinable())
    {
        // We cannot easily stop rclcpp::spin, but we can join the thread if the node is destroyed
        // Ideally, we should cancel the spin, but rclcpp::spin blocks.
        // However, since we are destroying the object, the node might be destroyed too.
        // Let's just join it.
        // Note: rclcpp::shutdown() might be needed if spin is blocking.
        // But calling shutdown here might affect other nodes.
        // For now, let's assume the node is stopped elsewhere or we detach if we can't stop it.
        // Actually, detaching is safer to avoid terminate if we can't stop it.
        // But let's try to join.
        // If the node is spinning, it won't return.
        // So we should probably detach node_thread_ if we can't signal it to stop.
        // But wait, on_deactivate joins it? No, on_deactivate joins it.
        // If on_deactivate wasn't called, we are here.
        // If we join a spinning thread, we hang.
        // So we should DETACH node_thread_ if it's still running.
        node_thread_.detach(); 
    }
}

std::vector<hardware_interface::StateInterface> RobotCarHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (const auto & joint : info_.joints) {
    if (joint.name.find("left") != std::string::npos) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "position", &hw_position_left_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "velocity", &hw_velocity_left_));
    } else if (joint.name.find("right") != std::string::npos) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "position", &hw_position_right_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, "velocity", &hw_velocity_right_));
    }
  }

  // IMU state interfaces
  if (use_imu_) {
      const std::string& sensor_name = info_.sensors[0].name;
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "orientation.x", &imu_orientation_[0]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "orientation.y", &imu_orientation_[1]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "orientation.z", &imu_orientation_[2]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "orientation.w", &imu_orientation_[3]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "angular_velocity.x", &imu_angular_velocity_[0]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "angular_velocity.y", &imu_angular_velocity_[1]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "angular_velocity.z", &imu_angular_velocity_[2]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "linear_acceleration.x", &imu_linear_acceleration_[0]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "linear_acceleration.y", &imu_linear_acceleration_[1]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "linear_acceleration.z", &imu_linear_acceleration_[2]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotCarHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & joint : info_.joints) {
    if (joint.name.find("left") != std::string::npos) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "velocity", &hw_command_velocity_left_));
    } else if (joint.name.find("right") != std::string::npos) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "velocity", &hw_command_velocity_right_));
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RobotCarHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "Activating ...please wait...");
    try
    {
        serial_port_.Open(serial_port_name_);
         LibSerial::BaudRate baud_rate;
         switch (serial_baud_rate_) {
             case 9600: baud_rate = LibSerial::BaudRate::BAUD_9600; break;
             case 19200: baud_rate = LibSerial::BaudRate::BAUD_19200; break;
             case 38400: baud_rate = LibSerial::BaudRate::BAUD_38400; break;
             case 57600: baud_rate = LibSerial::BaudRate::BAUD_57600; break;
             case 115200: baud_rate = LibSerial::BaudRate::BAUD_115200; break;
             case 230400: baud_rate = LibSerial::BaudRate::BAUD_230400; break;
             default:
                 RCLCPP_ERROR(rclcpp::get_logger("RobotCarHardwareInterface"),
                            "Unsupported baud rate: %d. Using 115200 as default.", serial_baud_rate_);
                 baud_rate = LibSerial::BaudRate::BAUD_115200;
                 break;
         }
        serial_port_.SetBaudRate(baud_rate);
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "Main serial port opened successfully: %s", serial_port_name_.c_str());
    }
    catch (const LibSerial::OpenFailed& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("RobotCarHardwareInterface"), "Failed to open serial port %s: %s", serial_port_name_.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize IMU serial port
    if (use_imu_) {
        try
        {
            RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "Attempting to open IMU serial port: %s at %d baud", imu_port_name_.c_str(), imu_baud_rate_);
            imu_serial_port_.Open(imu_port_name_);
            
            LibSerial::BaudRate imu_baud;
            // switch (imu_baud_rate_) {
            //     case 9600:   imu_baud = LibSerial::BaudRate::BAUD_9600; break;
            //     case 115200: imu_baud = LibSerial::BaudRate::BAUD_115200; break;
            //     default:
            //         RCLCPP_WARN(rclcpp::get_logger("RobotCarHardwareInterface"),
            //                    "Unsupported IMU baud rate: %d. Using 115200 as default.", imu_baud_rate_);
            //         imu_baud = LibSerial::BaudRate::BAUD_115200;
            //         break;
            // }
            imu_baud = LibSerial::BaudRate::BAUD_115200;
            imu_serial_port_.SetBaudRate(imu_baud);
            imu_serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            imu_serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            imu_serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
            imu_serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            imu_serial_port_.FlushIOBuffers();

            // Start IMU reading thread
            imu_thread_running_.store(true);
            imu_thread_ = std::thread(&RobotCarHardwareInterface::imuReadThread, this);
            RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "✅ IMU serial port opened and thread started: %s", imu_port_name_.c_str());
        }
        catch (const LibSerial::OpenFailed& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotCarHardwareInterface"), "❌ Failed to open IMU serial port %s: %s", imu_port_name_.c_str(), e.what());
            RCLCPP_WARN(rclcpp::get_logger("RobotCarHardwareInterface"), "IMU will not be available, but system will continue without it");
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotCarHardwareInterface"), "❌ IMU initialization error: %s", e.what());
            RCLCPP_WARN(rclcpp::get_logger("RobotCarHardwareInterface"), "IMU will not be available, but system will continue without it");
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotCarHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "Deactivating ...please wait...");

    // Stop IMU thread
    if (imu_thread_running_.load())
    {
        imu_thread_running_.store(false);
        if (imu_thread_.joinable())
        {
            imu_thread_.join();
        }
    }

    // Close serial ports
    if (serial_port_.IsOpen())
    {
        serial_port_.Close();
    }
    if (use_imu_ && imu_serial_port_.IsOpen())
    {
        imu_serial_port_.Close();
    }
    if (node_thread_.joinable())
    {
        node_thread_.join();
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotCarHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    static int read_counter = 0;
    read_counter++;

    // 1. Read all available data into the buffer
    if (serial_port_.IsDataAvailable())
    {
        try
        {
            std::vector<uint8_t> new_data;
            serial_port_.Read(new_data, 0, 0); // Read whatever is available
            serial_buffer_.insert(serial_buffer_.end(), new_data.begin(), new_data.end());
            
            // Debug: Print received data size
            // if (read_counter % 20 == 0) {
            //     RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), 
            //         "Received %zu bytes. Buffer size: %zu", new_data.size(), serial_buffer_.size());
            // }
        }
        catch (const LibSerial::ReadTimeout&) { /* Ignore */ }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotCarHardwareInterface"), 
                "Exception during serial read: %s", e.what());
        }
    }

    // Debug: Print buffer content if it's growing too large (stuck)
    if (serial_buffer_.size() > 100 && read_counter % 50 == 0) {
        std::stringstream ss;
        ss << "Buffer stuck? First 10 bytes: ";
        for (size_t i = 0; i < std::min<size_t>(10, serial_buffer_.size()); ++i) {
            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(serial_buffer_[i]) << " ";
        }
        RCLCPP_WARN(rclcpp::get_logger("RobotCarHardwareInterface"), "%s", ss.str().c_str());
    }

    // 2. Process buffer: Look for header [0xA0, 0x0A, 0x55]
    // We need at least sizeof(REPORT_DATA) bytes to process a packet
    while (serial_buffer_.size() >= sizeof(REPORT_DATA))
    {
        // Check for header
        if (serial_buffer_[0] == 0xA0 && 
            serial_buffer_[1] == 0x0A && 
            serial_buffer_[2] == 0x55)
        {
            // Header found, copy packet
            REPORT_DATA report_data;
            memcpy(&report_data, serial_buffer_.data(), sizeof(REPORT_DATA));

            // Verify checksum
            uint8_t checksum = CalcChecksum(serial_buffer_.data(), sizeof(REPORT_DATA));
            if (checksum == report_data.Sum)
            {
                // Valid packet, process it
                double robot_vx = static_cast<double>(report_data.Speed_X) / 1000.0;
                double robot_vth = static_cast<double>(report_data.Speed_Z) / 1000.0;
                uint8_t hand_cap = static_cast<uint8_t>(report_data.flag_1);

                hw_velocity_left_ = (robot_vx - robot_vth * wheel_separation_ / 2.0) / wheel_radius_;
                hw_velocity_right_ = (robot_vx + robot_vth * wheel_separation_ / 2.0) / wheel_radius_;

                hw_position_left_ += hw_velocity_left_ * period.seconds();
                hw_position_right_ += hw_velocity_right_ * period.seconds();

                auto msg = std::make_unique<robot_base::msg::CarInfo>();
                msg->speed_x = robot_vx;
                msg->speed_z = robot_vth;
                msg->power = report_data.power;
                msg->hand_capture = hand_cap;
                car_info_pub_->publish(std::move(msg));
                
                // Debug: Log successful publish
                // if (read_counter % 50 == 0) {
                //    RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "Published CarInfo: vx=%.2f", robot_vx);
                // }

                // Remove processed packet from buffer
                serial_buffer_.erase(serial_buffer_.begin(), serial_buffer_.begin() + sizeof(REPORT_DATA));
            }
            else
            {
                // Checksum failed, remove header and continue search
                RCLCPP_WARN(rclcpp::get_logger("RobotCarHardwareInterface"), 
                    "Checksum mismatch! Expected: %02x, Calc: %02x", report_data.Sum, checksum);
                serial_buffer_.erase(serial_buffer_.begin()); 
            }
        }
        else
        {
            // Header not found, remove first byte and continue search
            serial_buffer_.erase(serial_buffer_.begin());
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotCarHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    double vx = (hw_command_velocity_right_ + hw_command_velocity_left_) * wheel_radius_ / 2.0;
    double vth = (hw_command_velocity_right_ - hw_command_velocity_left_) * wheel_radius_ / wheel_separation_;
    
    // Debug log for write (throttled)
    static int write_counter = 0;
    write_counter++;
    // if (write_counter % 20 == 0) {
    //     RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), 
    //         "write() called. cmd_vel_L=%.3f, cmd_vel_R=%.3f -> vx=%.3f, vth=%.3f", 
    //         hw_command_velocity_left_, hw_command_velocity_right_, vx, vth);
    // }

    CMD_DATA cmd_data;
    cmd_data.Head_1 = 0xA0;
    cmd_data.Head_2 = 0x0A;
    cmd_data.cmd_1  = 0xAA;
    cmd_data.cmd_2  = 0x20;
    cmd_data.mode1   = hw_mode1_.load();
    cmd_data.mode2   = hw_mode2_.load();
    cmd_data.Speed_X = static_cast<int>(vx * 1000);
    cmd_data.Speed_Y = 0;
    cmd_data.Speed_Z = static_cast<int>(vth * 1000);
    cmd_data.Sum = CalcChecksum(reinterpret_cast<uint8_t*>(&cmd_data), sizeof(CMD_DATA));
    
    std::vector<uint8_t> write_buffer(sizeof(CMD_DATA));
    memcpy(write_buffer.data(), &cmd_data, sizeof(CMD_DATA));

    try {
        serial_port_.Write(write_buffer);
        
        // if (write_counter % 20 == 0) {
        //      std::stringstream ss;
        //      ss << "Write bytes: ";
        //      for (size_t i = 0; i < write_buffer.size(); ++i) {
        //          ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(write_buffer[i]) << " ";
        //      }
        //      RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "%s", ss.str().c_str());
        // }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("RobotCarHardwareInterface"), 
            "Exception during serial write: %s", e.what());
    }

    return hardware_interface::return_type::OK;
}

// IMU data reading thread (FIXED to handle timeouts gracefully)
void RobotCarHardwareInterface::imuReadThread()
{
    std::vector<uint8_t> buffer;
    RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "🔄 IMU reading thread started");

    while (imu_thread_running_.load())
    {
        try
        {
            std::vector<uint8_t> new_data;
            imu_serial_port_.Read(new_data, 64, 200); // Read up to 64 bytes, wait max 200ms

            if (!new_data.empty())
            {
                buffer.insert(buffer.end(), new_data.begin(), new_data.end());
            }
        }
        catch (const LibSerial::ReadTimeout&)
        {
            // Ignore timeout
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotCarHardwareInterface"), "❌ IMU read thread error: %s", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Parse packets from buffer
        while (buffer.size() >= 11)
        {
            if (buffer[0] == 0x55)
            {
                std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + 11);

                if (imuChecksumOk(packet))
                {
                    processImuPacket(packet);
                }
                // else
                // {
                //     // ✅ 打印非法 packet 数据
                //     std::stringstream ss;
                //     ss << "❌ Bad IMU checksum. Packet: ";
                //     for (auto byte : packet)
                //     {
                //         ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                //     }
                //     RCLCPP_ERROR(rclcpp::get_logger("RobotCarHardwareInterface"), "%s", ss.str().c_str());
                // }

                buffer.erase(buffer.begin(), buffer.begin() + 11);
            }
            else
            {
                // // ✅ 打印整个 buffer 前几个字节，帮助你诊断
                // std::stringstream ss;
                // ss << "❌ IMU header mismatch. Buffer start: ";
                // size_t print_len = std::min<size_t>(16, buffer.size());
                // for (size_t i = 0; i < print_len; ++i)
                // {
                //     ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]) << " ";
                // }
                // RCLCPP_ERROR(rclcpp::get_logger("RobotCarHardwareInterface"), "%s", ss.str().c_str());

                buffer.erase(buffer.begin());
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("RobotCarHardwareInterface"), "🛑 IMU reading thread stopped");
}


// Process IMU data packet
void RobotCarHardwareInterface::processImuPacket(const std::vector<uint8_t>& packet)
{
    if (packet.size() < 11) return;

    // Lock the mutex to safely update shared data
    std::lock_guard<std::mutex> lock(imu_data_mutex_); 

    uint8_t data_type = packet[1];
    switch (data_type)
    {
        // Acceleration
        case 0x51: 
        {
            int16_t ax_raw = (packet[3] << 8) | packet[2];
            int16_t ay_raw = (packet[5] << 8) | packet[4];
            int16_t az_raw = (packet[7] << 8) | packet[6];
            // Convert to m/s² (range ±16g)
            imu_linear_acceleration_[0] = static_cast<double>(ax_raw) / 32768.0 * 16.0 * 9.80665;
            imu_linear_acceleration_[1] = static_cast<double>(ay_raw) / 32768.0 * 16.0 * 9.80665;
            imu_linear_acceleration_[2] = static_cast<double>(az_raw) / 32768.0 * 16.0 * 9.80665;
            break;
        }
        // Angular Velocity
        case 0x52:
        {
            int16_t wx_raw = (packet[3] << 8) | packet[2];
            int16_t wy_raw = (packet[5] << 8) | packet[4];
            int16_t wz_raw = (packet[7] << 8) | packet[6];
            // Convert to rad/s (range ±2000°/s)
            imu_angular_velocity_[0] = static_cast<double>(wx_raw) / 32768.0 * 2000.0 * M_PI / 180.0;
            imu_angular_velocity_[1] = static_cast<double>(wy_raw) / 32768.0 * 2000.0 * M_PI / 180.0;
            imu_angular_velocity_[2] = static_cast<double>(wz_raw) / 32768.0 * 2000.0 * M_PI / 180.0;
            //             RCLCPP_INFO(rclcpp::get_logger("imu"), "ang_vel: [%f %f %f]",
            // imu_angular_velocity_[0], imu_angular_velocity_[1], imu_angular_velocity_[2]);
            break;
        }
        // Euler Angles
        case 0x53:
        {
            // std::cout << "Packet: ";
            // for (size_t i = 0; i < packet.size(); ++i)  // 如果 packet 是 std::vector<uint8_t>
            // {
            //     std::cout << "0x" 
            //             << std::uppercase << std::hex << std::setw(2) << std::setfill('0') 
            //             << static_cast<int>(packet[i]) << " ";
            // }
            // std::cout << std::dec << std::endl; // 恢复为十进制

            int16_t roll_raw = (packet[3] << 8) | packet[2];
            int16_t pitch_raw = (packet[5] << 8) | packet[4];
            int16_t yaw_raw = (packet[7] << 8) | packet[6];
            // Convert to degrees (range ±180°)
            double roll_deg = static_cast<double>(roll_raw) / 32768.0 * 180.0;
            double pitch_deg = static_cast<double>(pitch_raw) / 32768.0 * 180.0;
            double yaw_deg = static_cast<double>(yaw_raw) / 32768.0 * 180.0;

            // std::cout << "yaw_raw: " << yaw_raw 
            //   << "  yaw_deg: " << yaw_deg << std::endl;
            // Convert to radians
            double roll_rad = roll_deg * M_PI / 180.0;
            double pitch_rad = pitch_deg * M_PI / 180.0;
            double yaw_rad = yaw_deg * M_PI / 180.0;
            // Calculate quaternion from Euler angles and update
            imu_orientation_ = quaternionFromEuler(roll_rad, pitch_rad, yaw_rad);
            break;
        }
        // Magnetometer
        case 0x54:
        {
            int16_t mx_raw = (packet[3] << 8) | packet[2];
            int16_t my_raw = (packet[5] << 8) | packet[4];
            int16_t mz_raw = (packet[7] << 8) | packet[6];
            hw_magnetic_field_[0] = static_cast<double>(mx_raw);
            hw_magnetic_field_[1] = static_cast<double>(my_raw);
            hw_magnetic_field_[2] = static_cast<double>(mz_raw);

            // Publish magnetometer message directly from this thread
            auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();
            mag_msg->header.stamp = rclcpp::Clock().now();
            mag_msg->header.frame_id = imu_frame_id_;
            mag_msg->magnetic_field.x = hw_magnetic_field_[0];
            mag_msg->magnetic_field.y = hw_magnetic_field_[1];
            mag_msg->magnetic_field.z = hw_magnetic_field_[2];
            mag_msg->magnetic_field_covariance.fill(0.0); 
            mag_pub_->publish(std::move(mag_msg));
            break;
        }
    }


}

// This function is no longer used but is kept to avoid breaking the class definition.
void RobotCarHardwareInterface::processHWT901BData(const std::vector<uint8_t>& /*buffer*/)
{
   // This function can be left empty.
}

}  // namespace robotcar_base

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robotcar_base::RobotCarHardwareInterface, hardware_interface::SystemInterface)
