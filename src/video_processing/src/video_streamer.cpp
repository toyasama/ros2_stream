#include "video_processing/video_streamer.hpp"
#include "rclcpp/rclcpp.hpp"

VideoStreamer::VideoStreamer(int width, int height, const std::string& ffmpeg_command)
    : m_width(width), m_height(height) {
    
    ffmpeg_pipe = popen(ffmpeg_command.c_str(), "w");
    if (!ffmpeg_pipe) {
        RCLCPP_ERROR(rclcpp::get_logger("video processing"), "Failed to launch ffmpeg process");
        return;
    }
}

bool VideoStreamer::isReady() const {
    return ffmpeg_pipe != nullptr;
}

VideoStreamer::~VideoStreamer() {
    if (ffmpeg_pipe) {
        pclose(ffmpeg_pipe);
    }
}

bool VideoStreamer::writeFrame(const cv::Mat& frame) {
    if (!ffmpeg_pipe || frame.empty()) return false;
    if (frame.cols != m_width || frame.rows != m_height) return false;

    size_t written = fwrite(frame.data, 1, m_width * m_height * 3, ffmpeg_pipe);
    return written == (size_t)(m_width * m_height * 3);
}