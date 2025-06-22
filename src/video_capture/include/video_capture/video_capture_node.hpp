#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "video_capture/capture.hpp"
#include "video_capture/video_capture_params.hpp"

namespace video_capture {
    class VideoCaptureNode : public rclcpp::Node {
        public:
            VideoCaptureNode(const rclcpp::NodeOptions & options);

        private:
            void captureFrame();
            VideoCaptureParams m_params;
            Capture m_capture;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_pub;
            rclcpp::TimerBase::SharedPtr m_timer;
    };
}
