cmake_minimum_required(VERSION 3.8)
project(video_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(video_capture_lib
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
  image_transport
  cv_bridge
  sensor_msgs
  OpenCV
)

add_executable(video_capture_node src/main.cpp)
target_link_libraries(video_capture_node video_capture_lib)

install(TARGETS
  video_capture_lib
  video_capture_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
