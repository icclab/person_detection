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
from launch.actions import ExecuteProcess

def generate_launch_description():

    person_detect_pkg = get_package_share_directory("person_detect")
    compress_level = LaunchConfiguration("compress")

    launch_dir = os.path.dirname(os.path.realpath(__file__))
    default_bag = os.path.abspath(os.path.join(launch_dir, '../../../rosbags/oakimageraw_1Hz'))
    # default_bag = os.path.abspath(os.path.join(launch_dir, '../../../rosbags/oakimageraw_5Hz'))
    # default_bag = os.path.abspath(os.path.join(launch_dir, '../../../rosbags/oakimageraw_10Hz'))
    bag_path = LaunchConfiguration("bag_path")

    return LaunchDescription([

        # Declare the 'compress' launch argument
        DeclareLaunchArgument(
            "compress",
            default_value="80",
            description="Compression level to use"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(person_detect_pkg, "launch", "compress.launch.py")
            ),
            launch_arguments={"compress": compress_level}.items(),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(person_detect_pkg, "launch", "orin_rgb.launch.py")
        #     )),
                
        DeclareLaunchArgument(
            'bag_path',
            default_value=default_bag,
            description='Path to ROS 2 bag directory'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--delay', '5'],
            output='screen'
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
