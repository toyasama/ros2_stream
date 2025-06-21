#include "video_capture/video_capture_params.hpp"


namespace video_capture{
    video_capture_params::video_capture_params(rclcpp::Node* node){

        node->declare_parameter<std::string>("device_path", "/dev/video0");
        node->declare_parameter<int>("width", 640);
        node->declare_parameter<int>("height", 480);
        node->declare_parameter<int>("fps", 20);

        node->get_parameter("device_path", c_info.device_path);
        node->get_parameter("width",  c_info.width);
        node->get_parameter("height", c_info.height);
        node->get_parameter("fps", c_info.fps);

    }
}