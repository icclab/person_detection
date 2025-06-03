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
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    compress_level = LaunchConfiguration("compress")
    image_folder = LaunchConfiguration('image_folder')
    csv_file = LaunchConfiguration('csv_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'image_folder',
            default_value='/home/icc-nano/energy_ws/src/test_person_1/',
            description='Folder containing image files to publish'
        ),

        DeclareLaunchArgument(
            'compress',
            default_value='60',
            description='compress'
        ),

        DeclareLaunchArgument(
            'csv_file',
            default_value='/home/icc-nano/energy_ws/src/workload_1.csv',
            description='Folder containing image files to publish'
        ),

        Node(
            package='pub_yolo',
            executable='dynamic_freq',
            name='image_publisher',
            namespace='oak',
            parameters=[{"use_sim_time": False},                 
                {'image_folder': image_folder},
                {'csv_file': csv_file},
                {'compress': compress_level},],
            output='screen',
            # remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static'),],
            emulate_tty=True,
        ),
    ])
