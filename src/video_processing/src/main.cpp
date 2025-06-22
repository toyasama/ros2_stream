#include "video_processing/video_processing_node.hpp"
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<video_processing::VideoProcessingNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}