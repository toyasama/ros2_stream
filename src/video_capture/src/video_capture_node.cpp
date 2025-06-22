#include "video_capture/video_capture_node.hpp"
#include "cv_bridge/cv_bridge.h"

namespace video_capture {

    VideoCaptureNode::VideoCaptureNode():  Node("video_capture_node"),m_params(this), m_capture(m_params.camera_config) {

        if (!m_capture.isActive()){
            RCLCPP_FATAL(this->get_logger(), "Camera init failed: capture is not active");
            rclcpp::shutdown();
            return;
        }
        m_image_pub = image_transport::create_publisher(this, m_params.image_topic_pub, rmw_qos_profile_sensor_data);
        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000 / m_params.camera_config.fps), std::bind(&VideoCaptureNode::captureFrame, this));

    }


    void VideoCaptureNode::captureFrame() {
        auto image = m_capture.getImage();
        if (!image) {
            RCLCPP_ERROR(this->get_logger(), "No capture");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.value()).toImageMsg();
        msg->header.stamp = this->now();
        m_image_pub.publish(msg);
    }
}