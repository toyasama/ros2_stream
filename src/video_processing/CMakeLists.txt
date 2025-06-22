cmake_minimum_required(VERSION 3.8)
project(video_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(video_processing_lib
  src/video_processing_node.cpp
  src/filter.cpp
  src/overlay.cpp
)

target_include_directories(video_processing_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(video_processing_lib
  rclcpp
  cv_bridge
  OpenCV
  sensor_msgs
)

add_executable(video_processing_node src/main.cpp)
target_link_libraries(video_processing_node video_processing_lib)

install(TARGETS
  video_processing_lib
  video_processing_node
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
