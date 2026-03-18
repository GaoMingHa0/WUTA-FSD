cmake_minimum_required(VERSION 3.8)
project(line_detector)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)

add_executable(line_detector_node
  src/line_detector_node.cpp
)
target_include_directories(line_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(line_detector_node
  rclcpp geometry_msgs wuta_msgs
)

install(TARGETS line_detector_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
