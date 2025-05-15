from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    person_detect_pkg = get_package_share_directory("person_detect")

    use_cuda = LaunchConfiguration("use_cuda")

    return LaunchDescription([

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(person_detect_pkg, "launch", "orin_rgb.launch.py")
        #     )),

        DeclareLaunchArgument(
            'use_cuda',
            default_value='True',
            description='Use CUDA for inference'
        ),

        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play', bag_path, '--delay', '5'],
        #     output='screen'
        # ),

        Node(
            package='person_detect',
            executable='person_detect_driver_tinyv4',
            name='yolov4_tiny_node',
            namespace='oak',
            parameters=[{"use_sim_time": False}, {"use_cuda": use_cuda}],
            output='screen',
            # remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static'),],
            emulate_tty=True,
        ),

        Node(
            package='person_detect',
            executable='log_tegrastats',
            name='tegrastats_node',
            namespace='oak',
            parameters=[{"use_sim_time": False}],
            output='screen',
            # remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static'),],
            emulate_tty=True,
        ),

        # Delayed inclusion of another launch file (e.g., orin_rgb.launch.py)
        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(person_detect_pkg, "launch", "img_pub_freq_5.launch.py")
                    )
                )
            ]
        ),
    ])
