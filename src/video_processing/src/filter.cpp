#include "video_processing/filter.hpp"

namespace video_processing {

void Filter::applyHeatMap(cv::Mat& frame) {
    cv::Mat gray, color_map;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::applyColorMap(gray, color_map, cv::COLORMAP_JET);
    frame = color_map;
}

}
