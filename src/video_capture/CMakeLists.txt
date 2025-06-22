cmake_minimum_required(VERSION 3.8)
project(video_capture)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(video_capture_lib SHARED
  src/video_capture_node.cpp
  src/capture.cpp
  src/video_capture_params.cpp
  src/camera_config.cpp
)

target_include_directories(video_capture_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(video_capture_lib
  rclcpp
  rclcpp_components
  image_transport
  cv_bridge
  sensor_msgs
  OpenCV
)

rclcpp_components_register_nodes(video_capture_lib "video_capture::VideoCaptureNode")

add_executable(video_capture_node src/main.cpp)
target_link_libraries(video_capture_node video_capture_lib)
ament_target_dependencies(video_capture_node rclcpp)

install(TARGETS
  video_capture_lib
  video_capture_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(video_capture_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
