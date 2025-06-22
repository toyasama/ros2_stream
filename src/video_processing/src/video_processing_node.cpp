
#include "video_processing/video_processing_node.hpp"
#include "video_processing/filter.hpp"
#include "video_processing/overlay.hpp"

#include "cv_bridge/cv_bridge.h"

namespace video_processing{

    VideoProcessingNode::VideoProcessingNode():Node("video_processing_node"){
        m_image_sub = create_subscription<sensor_msgs::msg::Image>("/image_raw", rclcpp::SensorDataQoS(), 
                        std::bind(&VideoProcessingNode::processImage, this, std::placeholders::_1));
        m_image_pub = create_publisher<sensor_msgs::msg::Image>("/image_processed", rclcpp::SensorDataQoS());
    }

    void VideoProcessingNode::processImage(const sensor_msgs::msg::Image::SharedPtr msg){
        auto start = this->now();


        m_time_buffer[m_time_index] = start;
        m_time_index = (m_time_index + 1) % BUFFER_SIZE;
        if (m_time_count < BUFFER_SIZE) m_time_count++;


        float avg_delta = computeMeanDelta();
        float fps = avg_delta > 0.0 ? 1.0f / avg_delta : 0.0f;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat& frame = cv_ptr->image;

        Filter::applyHeatMap(frame);


        rclcpp::Duration processing_time = this->now() - start;
        float process_sec = processing_time.seconds();

        Overlay::applyTextOverlay(frame, "Processing Node", process_sec, fps);

        cv_ptr->header.stamp = this->now();
        m_image_pub->publish(*cv_ptr->toImageMsg());
    }

    float VideoProcessingNode::computeMeanDelta() {
        if (m_time_count < 2) return 0.0f;

        double total = 0.0;
        for (size_t i = 1; i < m_time_count; ++i) {
            size_t idx_prev = (m_time_index + BUFFER_SIZE - i - 1) % BUFFER_SIZE;
            size_t idx_curr = (m_time_index + BUFFER_SIZE - i) % BUFFER_SIZE;
            total += (m_time_buffer[idx_curr] - m_time_buffer[idx_prev]).seconds();
        }

        return static_cast<float>(total / (m_time_count - 1));
    }
}