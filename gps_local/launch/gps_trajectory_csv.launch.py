from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    trajectory_filename = LaunchConfiguration('trajectory_filename', default='gps_trajectory.csv')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        DeclareLaunchArgument(
            'trajectory_filename',
            default_value=trajectory_filename,
            description='Name of the output trajectory file'),
        
        Node(
            package='gps_local',
            executable='gps_local_node',
            name='gps_local_node',
            namespace='summit',
            parameters=[{"use_sim_time": use_sim_time}],
            output='screen',
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
            emulate_tty=True,
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
                 'ParentFrame': 'enu',
                 'ChildFrame': 'gps_antenna'
                }
            ],
            remappings=[('/tf', '/summit/tf'), ('/tf_static', '/summit/tf_static')],
        )
    ]) 