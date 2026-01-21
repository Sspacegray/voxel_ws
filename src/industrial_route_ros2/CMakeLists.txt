cmake_minimum_required(VERSION 3.8)
project(industrial_route_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

find_package(industrial_route_core REQUIRED)
find_package(industrial_route_interfaces REQUIRED)

# Nav2 dependencies for planner plugin
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

# Industrial Route Server executable
add_executable(industrial_route_server
  src/industrial_route_server_node.cpp
  src/occupancy_grid_collision_checker.cpp
  src/pure_pursuit_controller.cpp
  src/rviz_markers.cpp
)
target_compile_features(industrial_route_server PUBLIC cxx_std_17)
target_include_directories(industrial_route_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(industrial_route_server
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  visualization_msgs
  rosidl_runtime_cpp
  industrial_route_interfaces
  industrial_route_core
)

# Industrial Route Planner plugin (Nav2 GlobalPlanner)
add_library(industrial_route_planner SHARED
  src/industrial_route_planner.cpp
)
target_compile_features(industrial_route_planner PUBLIC cxx_std_17)
target_include_directories(industrial_route_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(industrial_route_planner
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  tf2_ros
  pluginlib
  industrial_route_interfaces
)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS industrial_route_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS industrial_route_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/plan_and_execute.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(industrial_route_planner)
ament_export_dependencies(
  nav2_core
  nav2_costmap_2d
  pluginlib
)

ament_package()
