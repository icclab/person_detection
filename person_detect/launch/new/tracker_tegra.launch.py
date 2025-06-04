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

    target_gt_id = LaunchConfiguration("target_gt_id")
    desired_id = LaunchConfiguration("desired_id")
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'gt_folder',
            default_value='/home/icc-nano/energy_ws/src/MOT20-01/',
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
            executable='person_tracker',
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

        # # Declare the 'compress' launch argument
        # DeclareLaunchArgument(
        #     "compress",
        #     default_value="60",
        #     description="Compression level to use"
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(person_detect_pkg, "launch", "compress.launch.py")
        #     ),
        #     launch_arguments={"compress": compress_level}.items(),
        # ),

        # Delayed inclusion of another launch file (e.g., orin_rgb.launch.py)
        TimerAction(
            period=10.0,  # Delay in seconds
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(person_detect_pkg, "launch", "img_pub_freq_1.launch.py")
                    )
                )
            ]
        ),
    ])
