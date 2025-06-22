
#include "video_processing/video_processing_node.hpp"
#include "video_processing/filter.hpp"
#include "video_processing/overlay.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "cv_bridge/cv_bridge.h"

namespace video_processing{

    VideoProcessingNode::VideoProcessingNode(const rclcpp::NodeOptions & options):Node("video_processing_node", options), m_params(this){
        m_image_sub = create_subscription<sensor_msgs::msg::Image>(m_params.image_topic_sub, rclcpp::SensorDataQoS(), 
                        std::bind(&VideoProcessingNode::processImage, this, std::placeholders::_1));
        m_image_pub = create_publisher<sensor_msgs::msg::Image>(m_params.image_topic_pub, rclcpp::SensorDataQoS());
        
        std::ostringstream ffmpeg_cmd;
        ffmpeg_cmd << "ffmpeg -f rawvideo -pixel_format bgr24 "
                << "-video_size " << m_params.width << "x" << m_params.height << " "
                << "-framerate 25 -i - "
                << "-fflags nobuffer -flags low_delay -analyzeduration 0 -probesize 32 "
                << "-c:v libx264 -preset ultrafast -tune zerolatency "
                << "-pix_fmt yuv420p "
                << "-f rtsp rtsp://localhost:8554/livestream";


        m_streamer = std::make_unique<VideoStreamer>(
            m_params.width,
            m_params.height,
            ffmpeg_cmd.str()
        );

        RCLCPP_INFO(this->get_logger(), "VideoCaptureNode started with intra-process: %s",
    this->get_node_options().use_intra_process_comms() ? "true" : "false");

    }
void VideoProcessingNode::processImage(sensor_msgs::msg::Image::UniquePtr msg)
{
    auto start = this->now();

    m_time_buffer[m_time_index] = start;
    m_time_index = (m_time_index + 1) % BUFFER_SIZE;
    if (m_time_count < BUFFER_SIZE) m_time_count++;
    float avg_delta = computeMeanDelta();
    float fps = avg_delta > 0.0 ? 1.0f / avg_delta : 0.0f;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat& frame = cv_ptr->image;

    Filter::applyHeatMap(frame);

    auto end_time = this->now();
    float process_sec = (end_time - start).seconds();
    float latency = (end_time - msg->header.stamp).seconds() * 1000.0;
    Overlay::applyTextOverlay(frame, "Processing Node", process_sec, fps, latency);

    if (!m_streamer->isReady()) {
        RCLCPP_WARN(this->get_logger(), "Streamer not ready, skipping frame.");
    } else {
        if (!m_streamer->writeFrame(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to write frame to streamer.");
        }
    }

    auto output_msg = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg(*output_msg);
    output_msg->header.stamp = end_time; 

    m_image_pub->publish(std::move(output_msg));
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
RCLCPP_COMPONENTS_REGISTER_NODE(video_processing::VideoProcessingNode)