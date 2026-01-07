cmake_minimum_required(VERSION 3.8)
project(industrial_route_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/RouteNode.msg"
  "msg/RouteEdge.msg"
  "msg/RouteGraph.msg"
  "msg/RouteStatus.msg"
)

set(srv_files
  "srv/PlanRoute.srv"
  "srv/GetNearestNode.srv"
  "srv/UpdateEdge.srv"
)

set(action_files
  "action/ExecuteRoute.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

