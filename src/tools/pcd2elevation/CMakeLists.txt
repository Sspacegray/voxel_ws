cmake_minimum_required(VERSION 3.8)
project(pcd2elevation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
add_executable(pcd2elevation src/pcd2elevation.cpp)
ament_target_dependencies(pcd2elevation
  rclcpp
  PCL
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
  grid_map_ros
  grid_map_pcl
  grid_map_msgs
  grid_map_cv
)

target_include_directories(pcd2elevation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${tf2_eigen_INCLUDE_DIRS}
)

install(TARGETS
        pcd2elevation
        DESTINATION lib/${PROJECT_NAME})
install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
