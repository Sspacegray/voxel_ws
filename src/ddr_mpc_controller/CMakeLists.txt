cmake_minimum_required(VERSION 3.10)
project(ddr_mpc_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor QUIET)

add_library(${PROJECT_NAME} SHARED
  src/ddr_mpc_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

if(osqp_vendor_FOUND)
  ament_target_dependencies(${PROJECT_NAME} osqp_vendor)
  target_link_libraries(${PROJECT_NAME} osqp::osqp)
  target_compile_definitions(${PROJECT_NAME} PRIVATE DDR_MPC_HAS_OSQP=1)
else()
  message(WARNING "ddr_mpc_controller: osqp_vendor not found, falling back to internal solver")
endif()

pluginlib_export_plugin_description_file(nav2_core ddr_mpc_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ddr_mpc_controller
    test/test_mpc_utils.cpp)
  target_link_libraries(test_ddr_mpc_controller
    ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
