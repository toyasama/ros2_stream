#include "video_capture/video_capture_node.hpp"
#include "cv_bridge/cv_bridge.h"

namespace video_capture {

    VideoCaptureNode::VideoCaptureNode():  Node("video_capture_node"),params_m(this), capture_m(params_m.c_info) {
        image_pub_m = image_transport::create_publisher(this, "image_raw", rmw_qos_profile_sensor_data);
        timer_m = this->create_wall_timer(
            std::chrono::milliseconds(1000 / params_m.c_info.fps), std::bind(&VideoCaptureNode::captureFrame, this));
    }


    void VideoCaptureNode::captureFrame(){
        auto start = this->now();

        const auto image = capture_m.getImage();
        if (!image.has_value()){
            RCLCPP_ERROR(this->get_logger(), "No capture");
            return;
        }

        const auto now = this->now();
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.value()).toImageMsg();
        msg->header.stamp = now;
        image_pub_m.publish(msg);

        auto end = this->now();
        // RCLCPP_INFO(this->get_logger(), "captureFrame took %.2f ms", (end - start).seconds() * 1000.0);

    }
}