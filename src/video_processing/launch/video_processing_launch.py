from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_processing',
            executable='video_processing_node',
            name='video_processing_node',
        )
    ])