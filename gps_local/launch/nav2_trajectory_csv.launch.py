from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    trajectory_filename = LaunchConfiguration('trajectory_filename', default='nav2_trajectory.csv')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        DeclareLaunchArgument(
            'trajectory_filename',
            default_value=trajectory_filename,
            description='Name of the output trajectory file'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps_link',
            namespace='summit',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments = ['--x', '-0.255', '--y', '0.0', '--z', '0.275', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'gps'],
            remappings=[('/tf_static', '/summit/tf_static')],
        ),
        

        Node(
            package='gps_local',
            executable='trajectory_csv_node',
            name='trajectory_csv_node',
            namespace='summit',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'filename': trajectory_filename,
                 'ParentFrame': 'map',
                 'ChildFrame': 'gps'
                }
            ],
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
        )
    ]) 