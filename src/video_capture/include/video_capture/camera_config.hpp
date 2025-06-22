#pragma once
#include <string>
#include <opencv2/opencv.hpp>

namespace video_capture {
    struct CameraConfig {
        std::string device_path;
        int width;
        int height;
        int fps;

        void apply(cv::VideoCapture& cap) const;
        void printConfig(cv::VideoCapture& cap) const ;
    };
}
