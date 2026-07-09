cmake_minimum_required(VERSION 3.8)
project(path_generator)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)
find_package(autoware_msgs REQUIRED)

add_executable(path_generator_node src/path_generator_node.cpp)

target_include_directories(path_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(path_generator_node
  rclcpp
  geometry_msgs
  wuta_msgs
  autoware_msgs
)

install(TARGETS path_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
