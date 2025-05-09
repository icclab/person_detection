from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    person_detect_pkg = get_package_share_directory("person_detect")
    use_cuda = LaunchConfiguration("use_cuda")

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(person_detect_pkg, "launch", "decompress.launch.py")
            )),

        DeclareLaunchArgument(
            'use_cuda',
            default_value='True',
            description='Use CUDA for inference'
        ),

        Node(
            package='person_detect',
            executable='yolo_v4_rap',
            name='yolov4_node',
            namespace='oak',
            parameters=[
                {"use_sim_time": False},
                {"use_cuda": use_cuda},
            ],
            output='screen',
            emulate_tty=True,
        ),  
    ])
