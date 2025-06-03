from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
 
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_folder',
            default_value='/home/icc-nano/energy_ws/src/MOT20-01/img1/',
            description='Folder containing image files to publish'
        ),
        DeclareLaunchArgument(
            'frequency',
            default_value='10.0',
            description='Frequency (Hz) to publish images'
        ),

        DeclareLaunchArgument(
            'compress',
            default_value='100',
            description='compress'
        ),
        
        Node(
            package='pub_yolo',
            executable='image_pub',
            name='image_publisher',
            parameters=[
                {'image_folder': LaunchConfiguration('image_folder')},
                {'frequency': LaunchConfiguration('frequency')},
                {'compress': LaunchConfiguration('compress')},
            ],
            output='screen'
        )
    ])