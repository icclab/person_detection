from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_pub',  
            executable='tf_pub',  
            name='summit_tf_pub',
            output='screen',
            remappings=[
                ('/tf', '/summit/tf'),
                ('/tf_static', '/summit/tf_static'),
            ],
        ),
    ])
