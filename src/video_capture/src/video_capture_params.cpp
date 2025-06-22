#include "video_capture/video_capture_params.hpp"


namespace video_capture{
    VideoCaptureParams::VideoCaptureParams(rclcpp::Node* node){

        node->declare_parameter<std::string>("device_path", "/dev/video0");
        node->declare_parameter<int>("width", 640);
        node->declare_parameter<int>("height", 480);
        node->declare_parameter<int>("fps", 20);
        node->declare_parameter<std::string>("image_topic_pub", "image_raw");

        node->get_parameter("device_path", camera_config.device_path);
        node->get_parameter("width",  camera_config.width);
        node->get_parameter("height", camera_config.height);
        node->get_parameter("fps", camera_config.fps);
        node->get_parameter("image_topic_pub", image_topic_pub);

    }
}