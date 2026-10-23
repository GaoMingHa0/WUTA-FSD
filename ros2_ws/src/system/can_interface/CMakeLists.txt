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
find_package(autoware_msgs REQUIRED)
find_package(wuta_msgs REQUIRED)

# 设备层：单个 SocketCAN 套接字，双向收发（纯 SocketCAN，不依赖 rclcpp）
add_library(can_socket src/can_socket.cpp)
target_include_directories(can_socket PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(can_socket PUBLIC cxx_std_17)

# 节点层：编排收发
add_executable(can_interface_node src/can_interface.cpp src/mode_cache.cpp)
target_include_directories(can_interface_node PRIVATE include)
target_compile_features(can_interface_node PRIVATE cxx_std_17)
ament_target_dependencies(can_interface_node rclcpp std_msgs geometry_msgs autoware_msgs wuta_msgs)
target_link_libraries(can_interface_node can_socket)

install(TARGETS can_interface_node can_socket
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  # 用内存CanSocket替代设备层；执行真实ROS回调、编码和超时路径，不连接总线。
  add_executable(test_brake_bench_can test/test_brake_bench_can.cpp src/can_interface.cpp src/mode_cache.cpp)
  target_compile_definitions(test_brake_bench_can PRIVATE CAN_INTERFACE_NO_MAIN)
  target_include_directories(test_brake_bench_can PRIVATE include)
  target_compile_features(test_brake_bench_can PRIVATE cxx_std_17)
  ament_target_dependencies(test_brake_bench_can rclcpp std_msgs geometry_msgs autoware_msgs wuta_msgs)
  add_executable(test_receive_only_can test/test_receive_only_can.cpp src/can_interface.cpp src/mode_cache.cpp)
  target_compile_definitions(test_receive_only_can PRIVATE CAN_INTERFACE_NO_MAIN)
  target_include_directories(test_receive_only_can PRIVATE include)
  target_compile_features(test_receive_only_can PRIVATE cxx_std_17)
  ament_target_dependencies(test_receive_only_can rclcpp std_msgs geometry_msgs autoware_msgs wuta_msgs)
  add_test(NAME receive_only_can COMMAND test_receive_only_can)
  set_tests_properties(receive_only_can PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=216;ROS_LOCALHOST_ONLY=1")
  add_executable(test_brake_bench_prepare test/test_brake_bench_prepare.cpp src/can_interface.cpp src/mode_cache.cpp)
  target_compile_definitions(test_brake_bench_prepare PRIVATE CAN_INTERFACE_NO_MAIN)
  target_include_directories(test_brake_bench_prepare PRIVATE include)
  target_compile_features(test_brake_bench_prepare PRIVATE cxx_std_17)
  ament_target_dependencies(test_brake_bench_prepare rclcpp std_msgs geometry_msgs autoware_msgs wuta_msgs)
  add_test(NAME brake_bench_prepare COMMAND test_brake_bench_prepare)
  set_tests_properties(brake_bench_prepare PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=214;ROS_LOCALHOST_ONLY=1")
  # 前置0x603发送入口已暂时注释，保留其历史测试但不作为当前启用行为执行。
  set_tests_properties(brake_bench_prepare PROPERTIES DISABLED TRUE)
  add_test(NAME brake_bench_can COMMAND test_brake_bench_can)
  set_tests_properties(brake_bench_can PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=215;ROS_LOCALHOST_ONLY=1")
endif()

ament_package()
