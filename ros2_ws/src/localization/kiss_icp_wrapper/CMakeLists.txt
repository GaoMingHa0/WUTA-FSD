cmake_minimum_required(VERSION 3.8)
project(kiss_icp_wrapper)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wuta_msgs REQUIRED)

add_executable(kiss_odom_sanitizer_node
  src/kiss_odom_sanitizer_node.cpp
)
ament_target_dependencies(kiss_odom_sanitizer_node
  nav_msgs
  rclcpp
)

install(TARGETS kiss_odom_sanitizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
