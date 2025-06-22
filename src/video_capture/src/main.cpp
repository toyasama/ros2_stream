#include "video_capture/video_capture_node.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<video_capture::VideoCaptureNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}