from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
 
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_folder',
            default_value='/home/icc-nano/energy_ws/src/test_person/',
            description='Folder containing image files to publish'
        ),
        DeclareLaunchArgument(
            'frequency',
            default_value='1.0',
            description='Frequency (Hz) to publish images'
        ),
        DeclareLaunchArgument(
            'duration',
            default_value='200.0',
            description='Total duration (seconds) to run the publisher'
        ),
        Node(
            package='pub_yolo',
            executable='image_pub',
            name='image_publisher',
            parameters=[
                {'image_folder': LaunchConfiguration('image_folder')},
                {'frequency': LaunchConfiguration('frequency')},
                {'duration': LaunchConfiguration('duration')}
            ],
            output='screen'
        )
    ])