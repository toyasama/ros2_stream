#pragma once

#include <rclcpp/rclcpp.hpp>
#include "video_capture/capture.hpp"
#include "video_capture/video_capture_params.hpp"
#include "image_transport/image_transport.hpp"

namespace video_capture{
    class VideoCaptureNode: public rclcpp::Node {

        public:

            VideoCaptureNode();
            ~VideoCaptureNode() = default;

        private:
            void captureFrame();
            video_capture_params m_params;
            Capture m_capture;
            image_transport::Publisher m_image_pub;
            rclcpp::TimerBase::SharedPtr m_timer;
            
    };
}