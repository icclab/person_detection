from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
 
def generate_launch_description():

    person_detect_pkg = get_package_share_directory("person_detect")

    compress_level = LaunchConfiguration("compress")

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_folder',
            default_value='/home/icc-nano/energy_ws/src/MOT20-01/img1/',
            description='Folder containing image files to publish'
        ),
        DeclareLaunchArgument(
            'frequency',
            default_value='1.0',
            description='Frequency (Hz) to publish images'
        ),

        DeclareLaunchArgument(
            'fps',
            default_value='30.0',
            description='Frames per second for image publishing'
        ),

        DeclareLaunchArgument(
            'compress',
            default_value='80',
            description='compress'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(person_detect_pkg, "launch", "compress.launch.py")
            ),
            launch_arguments={"compress": compress_level}.items(),
        ),

        Node(
            package='pub_yolo',
            executable='image_pub',
            name='image_publisher',
            parameters=[
                {'image_folder': LaunchConfiguration('image_folder')},
                {'frequency': LaunchConfiguration('frequency')},
                {'compress': compress_level},
                {'fps': LaunchConfiguration('fps')}
            ],
            output='screen'
        )
    ])