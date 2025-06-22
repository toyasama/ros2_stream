#include "video_capture/video_capture_node.hpp"
#include "cv_bridge/cv_bridge.h"

namespace video_capture {

    VideoCaptureNode::VideoCaptureNode():  Node("video_capture_node"),m_params(this), m_capture(m_params.c_info) {
        m_image_pub = image_transport::create_publisher(this, "image_raw", rmw_qos_profile_sensor_data);
        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000 / m_params.c_info.fps), std::bind(&VideoCaptureNode::captureFrame, this));
    }


    void VideoCaptureNode::captureFrame(){

        const auto image = m_capture.getImage();
        if (!image.has_value()){
            RCLCPP_ERROR(this->get_logger(), "No capture");
            return;
        }

        cv::Mat frame = image.value();
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        msg->header.stamp = this->now();
        m_image_pub.publish(msg);

    }
}