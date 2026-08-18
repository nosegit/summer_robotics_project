from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    cube_size_arg = DeclareLaunchArgument(
        'cube_size', default_value='0.10',
        description='Parcel cube side length in meters'
    )
    debug_image_arg = DeclareLaunchArgument(
        'debug_image', default_value='false',
        description='Publish marker and cube-centre debug images'
    )
    face_vertical_threshold_arg = DeclareLaunchArgument(
        'face_vertical_threshold', default_value='0.65',
        description='Maximum optical-Y component for a horizontal face normal'
    )
    face_switch_hysteresis_arg = DeclareLaunchArgument(
        'face_switch_hysteresis', default_value='0.15',
        description='Camera-facing score margin before switching cube faces'
    )
    position_filter_enabled_arg = DeclareLaunchArgument(
        'position_filter_enabled', default_value='true',
        description='Enable cube-centre outlier rejection and EMA smoothing'
    )
    position_filter_alpha_arg = DeclareLaunchArgument(
        'position_filter_alpha', default_value='0.25',
        description='New-measurement weight for cube-centre smoothing'
    )
    position_outlier_threshold_arg = DeclareLaunchArgument(
        'position_outlier_threshold', default_value='0.04',
        description='Reject single-frame cube-centre jumps larger than this (m)'
    )
    position_filter_reset_timeout_arg = DeclareLaunchArgument(
        'position_filter_reset_timeout', default_value='0.5',
        description='Reset a stale cube position filter after this duration (s)'
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
            'cube_size': LaunchConfiguration('cube_size'),
            'debug_image': ParameterValue(
                LaunchConfiguration('debug_image'), value_type=bool
            ),
            'face_vertical_threshold': ParameterValue(
                LaunchConfiguration('face_vertical_threshold'), value_type=float
            ),
            'face_switch_hysteresis': ParameterValue(
                LaunchConfiguration('face_switch_hysteresis'), value_type=float
            ),
            'position_filter_enabled': ParameterValue(
                LaunchConfiguration('position_filter_enabled'), value_type=bool
            ),
            'position_filter_alpha': ParameterValue(
                LaunchConfiguration('position_filter_alpha'), value_type=float
            ),
            'position_outlier_threshold': ParameterValue(
                LaunchConfiguration('position_outlier_threshold'), value_type=float
            ),
            'position_filter_reset_timeout': ParameterValue(
                LaunchConfiguration('position_filter_reset_timeout'), value_type=float
            ),
        }]
    )

    return LaunchDescription([
        marker_size_arg,
        dictionary_name_arg,
        image_topic_arg,
        camera_info_topic_arg,
        cube_size_arg,
        debug_image_arg,
        face_vertical_threshold_arg,
        face_switch_hysteresis_arg,
        position_filter_enabled_arg,
        position_filter_alpha_arg,
        position_outlier_threshold_arg,
        position_filter_reset_timeout_arg,
        aruco_node
    ])

