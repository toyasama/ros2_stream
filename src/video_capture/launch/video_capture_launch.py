from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('video_capture'),
        'params',
        'video_capture_param.yaml'
    )

    return LaunchDescription([
        Node(
            package='video_capture',
            executable='video_capture_node',
            name='video_capture_node',
            parameters=[param_file]
        )
    ])