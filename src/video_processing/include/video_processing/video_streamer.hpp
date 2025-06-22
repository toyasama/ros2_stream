#pragma once
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <memory>

class VideoStreamer {
public:
    VideoStreamer(int width, int height, const std::string& ffmpeg_command);
    ~VideoStreamer();

    bool writeFrame(const cv::Mat& frame);
    bool isReady() const ;

private:
    FILE* ffmpeg_pipe;
    int m_width, m_height;
};