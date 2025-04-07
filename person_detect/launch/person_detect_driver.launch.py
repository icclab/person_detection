from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='person_detect',
            executable='person_detect_driver',
            name='person_detect_driver',
            namespace='summit',
            parameters=[{"use_sim_time": False}],
            output='screen',
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static'),],
            emulate_tty=True,
        ),
    ])
