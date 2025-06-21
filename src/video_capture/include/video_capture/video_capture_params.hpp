#pragma once
#include <rclcpp/rclcpp.hpp>

namespace video_capture{
    struct capture_info{
        std::string device_path;
        int width, height, fps;
    };

   struct video_capture_params{
        video_capture_params(rclcpp::Node* node);
        capture_info c_info;
   };

}