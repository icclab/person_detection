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

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(person_detect_pkg, "launch", "decompress.launch.py")
            )),

        DeclareLaunchArgument(
            'gt_folder',
            default_value='/home/ros/rap/energy_ws/src/',
            description='Folder containing gt file'
        ),

        DeclareLaunchArgument(
            'target_gt_id',
            default_value='3',
            description='target gt id'
        ),

        DeclareLaunchArgument(
            'desired_id',
            default_value='4',
            description='desired tracking id'
        ),

        Node(
            package='person_detect',
            executable='person_tracker_rap',
            name='person_tracker_node',
            namespace='oak',
            parameters=[{"use_sim_time": False},
                {'gt_folder': LaunchConfiguration('gt_folder')},
                {'target_gt_id': LaunchConfiguration('target_gt_id')},
                {'desired_id': LaunchConfiguration('desired_id')},
            ],
            output='screen',
            # remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static'),],
            emulate_tty=True,
        ),
    ])
