cmake_minimum_required(VERSION 3.8)
project(mission_manager)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)

add_executable(mission_manager_node
  src/mission_manager.cpp
)

target_include_directories(mission_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(mission_manager_node
  rclcpp
  std_msgs
  wuta_msgs
)

install(TARGETS mission_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
