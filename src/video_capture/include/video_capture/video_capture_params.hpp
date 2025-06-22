#pragma once
#include <rclcpp/rclcpp.hpp>
#include "video_capture/camera_config.hpp"
namespace video_capture{
   struct VideoCaptureParams{
        VideoCaptureParams(rclcpp::Node* node);
        CameraConfig camera_config;
        std::string image_topic_pub;
   };

}