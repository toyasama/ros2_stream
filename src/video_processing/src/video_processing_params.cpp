#include "video_processing/video_processing_params.hpp"


namespace video_processing{
    VideoProcessingParams::VideoProcessingParams(rclcpp::Node* node){

        node->declare_parameter<std::string>("image_topic_sub", "image_raw");
        node->declare_parameter<std::string>("image_topic_pub", "image_processed");
        node->declare_parameter<int>("width", 1280);
        node->declare_parameter<int>("height", 720);

        node->get_parameter("image_topic_sub", image_topic_sub);
        node->get_parameter("image_topic_pub", image_topic_pub);
        node->get_parameter("width",  width);
        node->get_parameter("height", height);

    }
}