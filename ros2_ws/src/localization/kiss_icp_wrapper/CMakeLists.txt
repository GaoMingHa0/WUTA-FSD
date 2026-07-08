cmake_minimum_required(VERSION 3.8)
project(kiss_icp_wrapper)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wuta_msgs REQUIRED)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# TODO: add sources and targets
ament_package()
