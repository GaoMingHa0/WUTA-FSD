cmake_minimum_required(VERSION 3.8)
project(camera_detection)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wuta_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
add_executable(stereo_detection_adapter_cpp src/stereo_detection_adapter_cpp.cpp)
ament_target_dependencies(stereo_detection_adapter_cpp rclcpp sensor_msgs wuta_msgs)
install(TARGETS stereo_detection_adapter_cpp DESTINATION lib/${PROJECT_NAME})
find_package(OpenCV QUIET COMPONENTS core imgproc)
find_package(CUDAToolkit QUIET)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/local/cuda-11.8/targets/x86_64-linux/include
        /usr/include/x86_64-linux-gnu)
find_library(TENSORRT_LIBRARY nvinfer
  HINTS /usr/local/cuda-11.8/targets/x86_64-linux/lib
        /usr/lib/x86_64-linux-gnu)
if(OpenCV_FOUND AND CUDAToolkit_FOUND AND TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY)
  add_executable(lwdetr_tensorrt_node src/lwdetr_tensorrt_node.cpp)
  target_include_directories(lwdetr_tensorrt_node PRIVATE
    ${TENSORRT_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(lwdetr_tensorrt_node
    ${OpenCV_LIBS} ${TENSORRT_LIBRARY} CUDA::cudart)
  ament_target_dependencies(lwdetr_tensorrt_node
    rclcpp sensor_msgs std_msgs wuta_msgs)
  get_filename_component(TENSORRT_LIBRARY_DIR ${TENSORRT_LIBRARY} DIRECTORY)
  set_target_properties(lwdetr_tensorrt_node PROPERTIES
    BUILD_RPATH "${TENSORRT_LIBRARY_DIR};${CUDAToolkit_LIBRARY_DIR}"
    INSTALL_RPATH "${TENSORRT_LIBRARY_DIR};${CUDAToolkit_LIBRARY_DIR}")
  install(TARGETS lwdetr_tensorrt_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "LW-DETR C++ node skipped: requires TensorRT, CUDA Toolkit and OpenCV; Python nodes remain available")
endif()
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/stereo_detection_adapter DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/yolov8_onnx_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/yolov8_node DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(stereo_depth test/test_stereo.py)
  ament_add_pytest_test(yolo_decode test/test_yolo.py)
endif()
ament_package()
