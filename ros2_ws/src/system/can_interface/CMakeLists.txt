cmake_minimum_required(VERSION 3.8)
project(can_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)

# 设备层：发送 / 接收（纯 SocketCAN，不依赖 rclcpp）
add_library(can_sender src/can_sender.cpp)
target_include_directories(can_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(can_sender PUBLIC cxx_std_17)

add_library(can_receiver src/can_receiver.cpp)
target_include_directories(can_receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(can_receiver PUBLIC cxx_std_17)

# 节点层：编排收发
add_executable(can_interface_node src/can_interface.cpp)
target_include_directories(can_interface_node PRIVATE include)
target_compile_features(can_interface_node PRIVATE cxx_std_17)
ament_target_dependencies(can_interface_node rclcpp std_msgs geometry_msgs wuta_msgs)
target_link_libraries(can_interface_node can_sender can_receiver)

install(TARGETS can_interface_node can_sender can_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
