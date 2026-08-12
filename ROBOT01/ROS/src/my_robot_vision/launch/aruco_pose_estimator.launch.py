from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.05',
        description='Side length of the ArUco marker in meters'
    )

    dictionary_name_arg = DeclareLaunchArgument(
        'dictionary_name',
        default_value='DICT_5X5_100',
        description='ArUco dictionary name (e.g. DICT_4X4_50, DICT_5X5_100, DICT_6X6_250)'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='camera/image_raw',
        description='Topic name for the incoming ROS Image'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='camera/camera_info',
        description='Topic name for the incoming CameraInfo'
    )

    # Node Configuration
    aruco_node = Node(
        package='my_robot_vision',
        executable='aruco_pose_estimator',
        name='aruco_pose_estimator',
        output='screen',
        parameters=[{
            'marker_size': LaunchConfiguration('marker_size'),
            'dictionary_name': LaunchConfiguration('dictionary_name'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        }]
    )

    return LaunchDescription([
        marker_size_arg,
        dictionary_name_arg,
        image_topic_arg,
        camera_info_topic_arg,
        aruco_node
    ])

