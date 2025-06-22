#pragma once
#include <opencv2/opencv.hpp>

namespace video_processing {

class Filter {
public:
    static void applyHeatMap(cv::Mat& frame);
};

} 

