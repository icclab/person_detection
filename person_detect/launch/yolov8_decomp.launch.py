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

def generate_launch_description():

    person_detect_pkg = get_package_share_directory("person_detect")

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(person_detect_pkg, "launch", "decompress.launch.py")
            )),

        Node(
            package='person_detect',
            executable='person_detect_driver_v8',
            name='yolov8n_node',
            namespace='oak',
            parameters=[{"use_sim_time": False}],
            output='screen',
            # remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static'),],
            emulate_tty=True,
        ),
    ])
