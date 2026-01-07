cmake_minimum_required(VERSION 3.8)
project(visual_perception)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Define dependencies list for cleaner target linking
set(DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  image_transport
  image_geometry
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  message_filters
)

# ==============================================================================
# Depth Processor Node
# ==============================================================================
add_library(depth_processor_component SHARED
  src/depth_processor.cpp
)
target_include_directories(depth_processor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(depth_processor_component ${DEPENDENCIES})
target_link_libraries(depth_processor_component ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
rclcpp_components_register_nodes(depth_processor_component "visual_perception::DepthProcessor")

add_executable(depth_processor_node src/depth_processor_main.cpp)
target_link_libraries(depth_processor_node depth_processor_component)
ament_target_dependencies(depth_processor_node ${DEPENDENCIES})

# ==============================================================================
# Depth to Laser Node
# ==============================================================================
add_library(depth_to_laser_component SHARED
  src/depth_to_laser.cpp
)
target_include_directories(depth_to_laser_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(depth_to_laser_component ${DEPENDENCIES})
target_link_libraries(depth_to_laser_component ${OpenCV_LIBRARIES})
rclcpp_components_register_nodes(depth_to_laser_component "visual_perception::DepthToLaser")

add_executable(depth_to_laser_node src/depth_to_laser_main.cpp)
target_link_libraries(depth_to_laser_node depth_to_laser_component)
ament_target_dependencies(depth_to_laser_node ${DEPENDENCIES})

# ==============================================================================
# Road Segmentation Node
# ==============================================================================
add_library(road_segmentation_component SHARED
  src/road_segmentation.cpp
)
target_include_directories(road_segmentation_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(road_segmentation_component ${DEPENDENCIES})
target_link_libraries(road_segmentation_component ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
rclcpp_components_register_nodes(road_segmentation_component "visual_perception::RoadSegmentation")

add_executable(road_segmentation_node src/road_segmentation_main.cpp)
target_link_libraries(road_segmentation_node road_segmentation_component)
ament_target_dependencies(road_segmentation_node ${DEPENDENCIES})

# ==============================================================================
# Install
# ==============================================================================
install(TARGETS
  depth_processor_component
  depth_to_laser_component
  road_segmentation_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  depth_processor_node
  depth_to_laser_node
  road_segmentation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# Testing
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
