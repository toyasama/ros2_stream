#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "video_processing/video_processing_params.hpp"

namespace video_processing{
    class VideoProcessingNode : public rclcpp::Node{
        public:
            VideoProcessingNode();
            ~VideoProcessingNode() = default;
        
        private:
            
            void processImage(const sensor_msgs::msg::Image::SharedPtr msg);
            float computeMeanDelta();
            
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_sub;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_pub;
            VideoProcessingParams m_params;


            static constexpr size_t BUFFER_SIZE = 10;
            std::array<rclcpp::Time, BUFFER_SIZE> m_time_buffer;
            size_t m_time_index = 0;
            size_t m_time_count = 0;
            
    };
}