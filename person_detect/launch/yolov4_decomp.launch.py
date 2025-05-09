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
            executable='yolo_v4_rap',
            name='yolov4_node',
            namespace='oak',
            # parameters=[
            #     {"use_sim_time": False},
            #     {"output_prefix": 'yolo_v8'},
            # ],
            output='screen',
            emulate_tty=True,
        ),  
    ])
