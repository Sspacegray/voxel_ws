#ifndef ROBOTCAR_BASE__SERIAL_PORT_HPP_
#define ROBOTCAR_BASE__SERIAL_PORT_HPP_

#include <string>
#include <vector>
#include <serial/serial.h> // From https://github.com/wjwwood/serial

#include "rclcpp/rclcpp.hpp"

// Define the structures for serial communication protocol based on car_serial.cpp
#pragma pack(push, 1)
struct ReportData
{
    uint8_t head1;
    uint8_t head2;
    uint8_t cmd1;
    uint8_t flag1;
    uint8_t flag2;
    int32_t speed_x; // mm/s
    int32_t speed_y; // unused
    int32_t speed_z; // mm/s, but used for angular z
    float power;
    uint8_t sum;
};

struct CommandData
{
    uint8_t head1 = 0xA0;
    uint8_t head2 = 0x0A;
    uint8_t cmd1 = 0xAA;
    uint8_t cmd2 = 0x20;
    uint16_t mode1 = 0;
    uint16_t mode2 = 0;
    int32_t speed_x = 0; // in mm/s
    int32_t speed_y = 0;
    int32_t speed_z = 0; // in mm/s for angular
    uint8_t sum = 0;
};
#pragma pack(pop)


class SerialPort
{
public:
    SerialPort(
        const std::string &port_name,
        int baud_rate,
        rclcpp::Logger logger
    );
    ~SerialPort();

    bool open();
    void close();
    bool isOpen() const;

    // Read robot state (vx, vth, power) from serial
    bool read(double &vx, double &vth, float &power);

    // Write velocity commands (vx, vth) to serial
    bool write(double vx, double vth);

private:
    uint8_t calculate_checksum(const uint8_t *p_buf, uint32_t length);

    rclcpp::Logger logger_;
    serial::Serial serial_;
    std::string port_name_;
    int baud_rate_;
};

#endif // ROBOTCAR_BASE__SERIAL_PORT_HPP_ 