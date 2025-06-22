from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def is_mediamtx_running():
    try:
        result = subprocess.run(['pgrep', '-f', 'mediamtx'], stdout=subprocess.PIPE)
        return result.returncode == 0
    except Exception:
        return False

def generate_launch_description():
    actions = []

    mediamtx_params_path = os.path.join(
        get_package_share_directory('ros_stream'),
        'config',
        'mediamtx.yaml'
    )
    if not is_mediamtx_running():
        actions.append(
            ExecuteProcess(
                cmd=['mediamtx', mediamtx_params_path], 
                output='screen'
            )
        )

    capture_params = os.path.join(
        get_package_share_directory('video_capture'),
        'params',
        'video_capture_param.yaml'
    )

    processing_params = os.path.join(
        get_package_share_directory('video_processing'),
        'params',
        'video_processing_params.yaml'
    )

    capture_node = ComposableNode(
        package='video_capture',
        plugin='video_capture::VideoCaptureNode',
        name='video_capture_node',
        parameters=[capture_params],
        extra_arguments=[{
            'use_intra_process_comms': True
        }]
    )

    processing_node = ComposableNode(
        package='video_processing',
        plugin='video_processing::VideoProcessingNode',
        name='video_processing_node',
        parameters=[processing_params],
        extra_arguments=[{
            'use_intra_process_comms': True
        }]
    )

    container = ComposableNodeContainer(
        name='video_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            capture_node,
            processing_node
        ],
        output='screen',
    )

    actions.append(container)

    return LaunchDescription(actions)
