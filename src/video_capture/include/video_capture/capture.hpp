#pragma once
#include <opencv2/opencv.hpp>
#include <optional>
#include "video_capture/video_capture_params.hpp"

namespace video_capture{
    class Capture{
        public:
            Capture(const capture_info& info);
            std::optional<cv::Mat> getImage();

        private:
            cv::VideoCapture m_video_capture;
    };

}