cmake_minimum_required(VERSION 3.8)
project(lidar_detection)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search kdtree)
find_package(pcl_conversions REQUIRED)

add_executable(lidar_detection_node
  src/lidar_detection_node.cpp
  src/traditional_detector.cpp
  src/dl_detector.cpp
)

target_include_directories(lidar_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lidar_detection_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(lidar_detection_node
  rclcpp
  sensor_msgs
  visualization_msgs
  wuta_msgs
  pcl_conversions
)

install(TARGETS lidar_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
