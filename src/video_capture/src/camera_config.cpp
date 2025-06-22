#include "video_capture/camera_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace video_capture {
    void CameraConfig::apply(cv::VideoCapture& cap) const {
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap.set(cv::CAP_PROP_FPS, fps);
    }
    void CameraConfig::printConfig(cv::VideoCapture& cap) const {

        double actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        double actual_fps = cap.get(cv::CAP_PROP_FPS);

        RCLCPP_INFO(
            rclcpp::get_logger("video capture"),
            "Camera initialized at %.0fx%.0f @ %.2f FPS",
            actual_width, actual_height, actual_fps
        );
    }
}
