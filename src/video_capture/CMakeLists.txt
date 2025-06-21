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

add_library(video_capture_lib
  src/video_capture_node.cpp
  src/capture.cpp
  src/video_capture_params.cpp
)


include_directories(include)

add_executable(video_capture_node src/main.cpp)
ament_target_dependencies(video_capture_lib
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

target_link_libraries(video_capture_node video_capture_lib)

ament_target_dependencies(video_capture_node
  rclcpp
)


install(TARGETS
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
