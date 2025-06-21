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
            video_capture_params params_m;
            Capture capture_m;
            image_transport::Publisher image_pub_m;
            rclcpp::TimerBase::SharedPtr timer_m;
            std::shared_ptr<rclcpp::Time> last_time_m;
            
    };
}