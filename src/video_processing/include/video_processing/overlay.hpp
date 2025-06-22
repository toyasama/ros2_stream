#pragma once

#include <opencv2/opencv.hpp>
#include <string>

namespace video_processing {

class Overlay {
public:
    static void applyTextOverlay(cv::Mat& frame, const std::string& text, float delay = -1, float fps = -1, float latency_ms = -1);
};

} 