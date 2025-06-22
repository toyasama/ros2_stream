#include "video_capture/capture.hpp"

#include <rclcpp/rclcpp.hpp>
namespace video_capture{
    Capture::Capture(const capture_info& info ){

        m_video_capture.open(info.device_path, cv::CAP_V4L2);
        if (!m_video_capture.isOpened()) {
            RCLCPP_ERROR(rclcpp::get_logger("video capture"), "Failed to open video device: %s", info.device_path.c_str());
            rclcpp::shutdown();
        }
        m_video_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        m_video_capture.set(cv::CAP_PROP_FRAME_WIDTH, info.width);
        m_video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, info.height);
        m_video_capture.set(cv::CAP_PROP_FPS, info.fps);

        double actual_width = m_video_capture.get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_height = m_video_capture.get(cv::CAP_PROP_FRAME_HEIGHT);
        double actual_fps = m_video_capture.get(cv::CAP_PROP_FPS);

        RCLCPP_INFO(
            rclcpp::get_logger("video capture"),
            "Camera initialized at %.0fx%.0f @ %.2f FPS",
            actual_width, actual_height, actual_fps
        );
    }

    std::optional<cv::Mat> Capture::getImage(){
        cv::Mat frame;
        m_video_capture.read(frame);
        if (frame.empty()) {
            return std::nullopt;
        }

        return std::make_optional(std::move(frame));
    }
}