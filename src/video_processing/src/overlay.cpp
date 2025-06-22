#include "video_processing/overlay.hpp"

namespace video_processing {

void Overlay::applyTextOverlay(cv::Mat& frame, const std::string& text, float delay, float fps, float latency_ms) {
    std::string full_text = text;

    if (delay > 0) {
        full_text += " | Process time (s): " + std::to_string(delay);
        full_text += " | FPS (Hz): " + std::to_string(fps);
        full_text += " | Latency (ms): " + std::to_string(latency_ms);
    }

    const int font_face = cv::FONT_HERSHEY_SIMPLEX;
    const double font_scale = 0.7;
    const int thickness = 2;
    const cv::Scalar color(255, 255, 255);  
    const cv::Point position(10, 30);

    int baseline = 0;
    cv::Size text_size = cv::getTextSize(full_text, font_face, font_scale, thickness, &baseline);
    cv::rectangle(frame, position + cv::Point(0, 5), position + cv::Point(text_size.width, -text_size.height - 5), cv::Scalar(0, 0, 0), cv::FILLED);

    cv::putText(frame, full_text, position, font_face, font_scale, color, thickness);
}

}