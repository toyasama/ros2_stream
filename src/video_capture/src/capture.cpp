#include "video_capture/capture.hpp"

#include <rclcpp/rclcpp.hpp>
namespace video_capture{
    Capture::Capture(const capture_info& info ){

        video_capture_m.open(info.device_path, cv::CAP_V4L2);
        if (!video_capture_m.isOpened()) {
            RCLCPP_ERROR(rclcpp::get_logger("video capture"), "Failed to open video device: %s", info.device_path.c_str());
            rclcpp::shutdown();
        }
        video_capture_m.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        video_capture_m.set(cv::CAP_PROP_FRAME_WIDTH, info.width);
        video_capture_m.set(cv::CAP_PROP_FRAME_HEIGHT, info.height);
        video_capture_m.set(cv::CAP_PROP_FPS, info.fps);

        double actual_width = video_capture_m.get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_height = video_capture_m.get(cv::CAP_PROP_FRAME_HEIGHT);
        double actual_fps = video_capture_m.get(cv::CAP_PROP_FPS);
        double actual_fourcc = video_capture_m.get(cv::CAP_PROP_FOURCC);

        RCLCPP_INFO(
            rclcpp::get_logger("video capture"),
            "Camera initialized at %.0fx%.0f @ %.2f FPS",
            actual_width, actual_height, actual_fp
        );
    }

    std::optional<cv::Mat> Capture::getImage(){
        cv::Mat frame;
        video_capture_m.read(frame);
        if (frame.empty()) {
            return std::nullopt;
        }

        return std::make_optional(std::move(frame));
    }
}