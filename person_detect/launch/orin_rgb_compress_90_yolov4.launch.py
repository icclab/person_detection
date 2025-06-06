from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    person_detect_pkg = get_package_share_directory("person_detect")
    compress_level = LaunchConfiguration("compress")

    return LaunchDescription([

        # Declare the 'compress' launch argument
        DeclareLaunchArgument(
            "compress",
            default_value="90",
            description="Compression level to use"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(person_detect_pkg, "launch", "compress.launch.py")
            ),
            launch_arguments={"compress": compress_level}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(person_detect_pkg, "launch", "orin_rgb.launch.py")
            )),

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

        Node(
            package='person_detect',
            executable='yolo_v4_sub',
            name='yolo_node_sub',
            namespace='oak',
            parameters=[{"use_sim_time": False}],
            output='screen',
            # remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static'),],
            emulate_tty=True,
        ),
    ])
