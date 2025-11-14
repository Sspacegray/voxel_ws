cmake_minimum_required(VERSION 3.8)
project(robotcar_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_path(LIBSERIAL_INCLUDE_DIR NAMES libserial/SerialPort.h)
find_library(LIBSERIAL_LIBRARY NAMES serial)
if(NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIBRARY)
  message(FATAL_ERROR "LibSerial library or headers not found. Please ensure 'libserial-dev' is installed.")
endif()
find_package(rclpy REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)   # 如果你想用 tf2_geometry_msgs.hpp

# Build custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CarInfo.msg"
  "srv/SetControlMode.srv"
)


# Build Lidar Filter node as a component
add_library(laser_filter_node SHARED
  src/laser_filter.cpp
)
target_include_directories(laser_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(laser_filter_node
  rclcpp
  sensor_msgs
  rclcpp_components
)
rclcpp_components_register_node(laser_filter_node PLUGIN "CLidarFilter" EXECUTABLE "laser_filter_node_exe")

# Build hardware interface plugin
add_library(robotcar_hardware_interface SHARED
  src/robotcar_hardware_interface.cpp
)
target_include_directories(robotcar_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${LIBSERIAL_INCLUDE_DIR}
)
ament_target_dependencies(robotcar_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_control
  tf2
  tf2_geometry_msgs 
)
target_link_libraries(robotcar_hardware_interface ${LIBSERIAL_LIBRARY})

# Add dependency on the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(robotcar_hardware_interface ${cpp_typesupport_target})

# Export plugin for ros2_control
pluginlib_export_plugin_description_file(hardware_interface src/robotcar_hardware.xml)

# Install targets
install(TARGETS
    robotcar_hardware_interface
    laser_filter_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY launch config urdf
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
        scripts/imu_node.py
        scripts/laser_calibration_node.py
        scripts/dynamic_laser_tf_publisher.py
        scripts/sync_laser_params.py
        scripts/laser_frame_remapper.py
        # scripts/test_imu_integration.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package() 