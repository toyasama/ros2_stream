cmake_minimum_required(VERSION 3.8)
project(video_processing)

# Enable C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(video_processing_lib SHARED
  src/video_processing_node.cpp
  src/filter.cpp
  src/overlay.cpp
  src/video_processing_params.cpp
  src/video_streamer.cpp
)

target_include_directories(video_processing_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(video_processing_lib
  rclcpp
  rclcpp_components
  cv_bridge
  OpenCV
  sensor_msgs
)

rclcpp_components_register_nodes(video_processing_lib
  "video_processing::VideoProcessingNode"
)

add_executable(video_processing_node src/main.cpp)
target_link_libraries(video_processing_node video_processing_lib)
ament_target_dependencies(video_processing_node
  rclcpp
)

install(TARGETS
  video_processing_lib
  video_processing_node
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
ament_export_libraries(video_processing_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
