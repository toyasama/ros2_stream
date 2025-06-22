#pragma once
#include <opencv2/opencv.hpp>
#include <optional>
#include "video_capture/camera_config.hpp"

namespace video_capture{
    class Capture{
        public:
            Capture(const CameraConfig& config);
            std::optional<cv::Mat> getImage();
            bool isActive() const ;

        private:
            cv::VideoCapture m_video_capture;
    };

}