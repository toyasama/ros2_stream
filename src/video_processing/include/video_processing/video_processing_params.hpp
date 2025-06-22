#pragma once
#include <rclcpp/rclcpp.hpp>
namespace video_processing{
   struct VideoProcessingParams{
        VideoProcessingParams(rclcpp::Node* node);
        std::string image_topic_sub;
        std::string image_topic_pub;
   };

}