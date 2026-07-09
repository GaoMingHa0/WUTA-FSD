cmake_minimum_required(VERSION 3.8)
project(ndt_localization)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

# NDT localization node
add_executable(ndt_localization_node
  src/ndt_localization.cpp
)
target_include_directories(ndt_localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(ndt_localization_node ${PCL_LIBRARIES})
ament_target_dependencies(ndt_localization_node
  rclcpp sensor_msgs geometry_msgs nav_msgs visualization_msgs wuta_msgs pcl_conversions
)

# Map saver node
add_executable(map_saver_node
  src/map_saver.cpp
)
target_include_directories(map_saver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(map_saver_node ${PCL_LIBRARIES})
ament_target_dependencies(map_saver_node
  rclcpp sensor_msgs nav_msgs std_msgs wuta_msgs pcl_conversions
)

install(TARGETS ndt_localization_node map_saver_node
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
