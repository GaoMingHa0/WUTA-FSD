cmake_minimum_required(VERSION 3.8)
project(controller)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)

add_executable(controller_node
  src/controller_node.cpp
  src/pure_pursuit.cpp
  src/twist_filter.cpp
)

target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(controller_node
  rclcpp
  geometry_msgs
  visualization_msgs
  autoware_msgs
  wuta_msgs
)

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
