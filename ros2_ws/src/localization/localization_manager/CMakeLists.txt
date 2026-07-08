cmake_minimum_required(VERSION 3.8)
project(localization_manager)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)

add_executable(localization_manager_node
  src/localization_manager.cpp
)

target_include_directories(localization_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(localization_manager_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  wuta_msgs
)

install(TARGETS localization_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
