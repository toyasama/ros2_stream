from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ffplay',
                '-fflags', 'nobuffer',
                '-flags', 'low_delay',
                '-framedrop',
                'rtsp://localhost:8554/livestream'
            ],
            output='screen'
        )
    ])
