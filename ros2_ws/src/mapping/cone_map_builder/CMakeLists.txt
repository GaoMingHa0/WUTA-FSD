cmake_minimum_required(VERSION 3.8)
project(cone_map_builder)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(cone_map_builder_node
  src/cone_map_builder.cpp
)

target_include_directories(cone_map_builder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cone_map_builder_node
  yaml-cpp
)

ament_target_dependencies(cone_map_builder_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  wuta_msgs
)

install(TARGETS cone_map_builder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
