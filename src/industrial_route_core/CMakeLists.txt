cmake_minimum_required(VERSION 3.8)
project(industrial_route_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/geojson_loader.cpp
  src/graph.cpp
  src/map_matching.cpp
  src/planner.cpp
  src/path_builder.cpp
  src/route_planner.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

add_executable(industrial_route_validate_graph
  src/validate_graph_main.cpp
)
target_compile_features(industrial_route_validate_graph PUBLIC cxx_std_17)
target_link_libraries(industrial_route_validate_graph ${PROJECT_NAME})
target_include_directories(industrial_route_validate_graph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS industrial_route_validate_graph
  DESTINATION lib/${PROJECT_NAME}
)

install(EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_graph_and_planner test/test_graph_and_planner.cpp)
  target_link_libraries(test_graph_and_planner ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
