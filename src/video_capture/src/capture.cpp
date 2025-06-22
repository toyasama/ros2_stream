#include "video_capture/capture.hpp"

#include <rclcpp/rclcpp.hpp>
namespace video_capture{
    Capture::Capture(const CameraConfig& config){

        m_video_capture.open(config.device_path, cv::CAP_V4L2);
        if (!m_video_capture.isOpened())  {
            RCLCPP_ERROR(rclcpp::get_logger("video capture"), "Failed to open video device: %s", config.device_path.c_str());
            return;
        }
        
        config.apply(m_video_capture);
        config.printConfig(m_video_capture);
    }

    bool Capture::isActive() const {
        return m_video_capture.isOpened();
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