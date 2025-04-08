import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # File 2 - Variables and Parameters
    depthai_examples_path = get_package_share_directory('depthai_examples')
    default_rviz = os.path.join(depthai_examples_path, 'rviz', 'pointCloud.rviz')
    default_resources_path = os.path.join(depthai_examples_path, 'resources')

    camera_model = LaunchConfiguration('camera_model', default='OAK-D')
    tf_prefix = LaunchConfiguration('tf_prefix', default='oak')
    base_frame = LaunchConfiguration('base_frame', default='oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame', default='oak-d-base-frame')

    cam_pos_x = LaunchConfiguration('cam_pos_x', default='0.0')
    cam_pos_y = LaunchConfiguration('cam_pos_y', default='0.0')
    cam_pos_z = LaunchConfiguration('cam_pos_z', default='0.0')
    cam_roll = LaunchConfiguration('cam_roll', default='0.0')
    cam_pitch = LaunchConfiguration('cam_pitch', default='0.0')
    cam_yaw = LaunchConfiguration('cam_yaw', default='0.0')

    camera_param_uri = LaunchConfiguration('camera_param_uri', default='package://depthai_examples/params/camera')
    sync_nn = LaunchConfiguration('sync_nn', default=True)
    subpixel = LaunchConfiguration('subpixel', default=True)
    nnName = LaunchConfiguration('nnName', default="x")
    resourceBaseFolder = LaunchConfiguration('resourceBaseFolder', default=default_resources_path)
    confidence = LaunchConfiguration('confidence', default=200)
    lrCheckTresh = LaunchConfiguration('lrCheckTresh', default=5)
    monoResolution = LaunchConfiguration('monoResolution', default='400p')
    fullFrameTracking = LaunchConfiguration('fullFrameTracking', default=False)

    # Launch arguments
    launch_args = [
        DeclareLaunchArgument('camera_model', default_value=camera_model),
        DeclareLaunchArgument('tf_prefix', default_value=tf_prefix),
        DeclareLaunchArgument('base_frame', default_value=base_frame),
        DeclareLaunchArgument('parent_frame', default_value=parent_frame),
        DeclareLaunchArgument('cam_pos_x', default_value=cam_pos_x),
        DeclareLaunchArgument('cam_pos_y', default_value=cam_pos_y),
        DeclareLaunchArgument('cam_pos_z', default_value=cam_pos_z),
        DeclareLaunchArgument('cam_roll', default_value=cam_roll),
        DeclareLaunchArgument('cam_pitch', default_value=cam_pitch),
        DeclareLaunchArgument('cam_yaw', default_value=cam_yaw),
        DeclareLaunchArgument('camera_param_uri', default_value=camera_param_uri),
        DeclareLaunchArgument('sync_nn', default_value=sync_nn),
        DeclareLaunchArgument('subpixel', default_value=subpixel),
        DeclareLaunchArgument('nnName', default_value=nnName),
        DeclareLaunchArgument('resourceBaseFolder', default_value=resourceBaseFolder),
        DeclareLaunchArgument('confidence', default_value=confidence),
        DeclareLaunchArgument('lrCheckTresh', default_value=lrCheckTresh),
        DeclareLaunchArgument('monoResolution', default_value=monoResolution),
        DeclareLaunchArgument('fullFrameTracking', default_value=fullFrameTracking),
    ]

    # Node from file 1 (liquid_detection)
    roboflow_node = Node(
        package='liquid_detection',
        executable='roboflow',
        name='roboflow_water',
        output='screen',
        namespace='summit',
        remappings=[
           ('/leakage_marker', '/summit/leakage_marker'),
           ('/leakage_annotated_image', '/summit/leakage_annotated_image'),
        ],
        parameters=[
            {'image_topic': '/summit/color/image'},
            {'gps_topic': '/summit/gps/fix'},
            {'local_topic': '/summit/base_pose'},
        ]
    )

    tracker_node = Node(
        package='depthai_examples',
        executable='tracker_yolov4_spatial_node',
        output='screen',
        namespace='summit',
        parameters=[
            {'tf_prefix': tf_prefix},
            {'camera_param_uri': camera_param_uri},
            {'sync_nn': sync_nn},
            {'nnName': nnName},
            {'resourceBaseFolder': resourceBaseFolder},
            {'monoResolution': monoResolution},
            {'fullFrameTracking': fullFrameTracking},
        ]
    )

    person_detect_node = Node(
        package='person_detect',
        executable='person_detect_driver',
        name='person_detect_driver',
        namespace='summit',
        parameters=[{"use_sim_time": False}],
        output='screen',
        remappings=[
            ('/tf', '/summit/tf'),
            ('/tf_static', '/summit/tf_static'),
        ],
        emulate_tty=True,
    )

    # Optional: Rviz node
    rviz_node = Node(
       package='rviz2',
       executable='rviz2',
       output='screen',
       arguments=['--display-config', default_rviz]
    )

    return LaunchDescription(launch_args + [
        roboflow_node,
        tracker_node,
        person_detect_node,
        #rviz_node
    ])
