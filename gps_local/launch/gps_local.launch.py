from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='gps_local',
            executable='gps_local_node',
            name='gps_local_node',
            namespace='summit',
            parameters=[{"use_sim_time": False}],
            output='screen',
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
            emulate_tty=True,
        ),
    ])