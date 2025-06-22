#include "video_capture/video_capture_node.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace video_capture {

    VideoCaptureNode::VideoCaptureNode(const rclcpp::NodeOptions & options):  Node("video_capture_node", options),m_params(this), m_capture(m_params.camera_config) {

        if (!m_capture.isActive()){
            throw std::runtime_error("Camera init failed: capture is not active");
            return;
        }
        m_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
            m_params.image_topic_pub,
            rclcpp::SensorDataQoS()
        );
        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000 / m_params.camera_config.fps), std::bind(&VideoCaptureNode::captureFrame, this));
        RCLCPP_INFO(this->get_logger(), "VideoCaptureNode started with intra-process: %s",
            this->get_node_options().use_intra_process_comms() ? "true" : "false");


    }


    void VideoCaptureNode::captureFrame() {
        auto image = m_capture.getImage();
        if (!image) {
            RCLCPP_ERROR(this->get_logger(), "No capture");
            return;
        }

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.value()).toImageMsg(*msg);

    msg->header.stamp = this->now();
    msg->header.frame_id = "camera";  

    m_image_pub->publish(std::move(msg));

    }

}
RCLCPP_COMPONENTS_REGISTER_NODE(video_capture::VideoCaptureNode)