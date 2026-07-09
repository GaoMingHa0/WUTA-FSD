cmake_minimum_required(VERSION 3.8)
project(boundary_detector)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)
find_package(autoware_msgs REQUIRED)

# HRT-D pathplanning algorithm library (pure C++, no ROS)
file(GLOB PATHPLANNING_SRCS thirdparty/pathplanning/src/*.cpp)

add_library(pathplanning STATIC ${PATHPLANNING_SRCS})
target_include_directories(pathplanning PUBLIC thirdparty/pathplanning/include include)

add_executable(boundary_detector_node src/boundary_detector_node.cpp)

target_include_directories(boundary_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  thirdparty/pathplanning/include
)

target_link_libraries(boundary_detector_node pathplanning)

ament_target_dependencies(boundary_detector_node
  rclcpp
  geometry_msgs
  visualization_msgs
  wuta_msgs
  autoware_msgs
)

install(TARGETS boundary_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
