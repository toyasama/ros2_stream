from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('video_processing'),
        'params',
        'video_processing_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='video_processing',
            executable='video_processing_node',
            name='video_processing_node',
            parameters=[param_file]
        )
    ])