"""Receive the host Pi camera RTP stream and publish standard ROS images."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_calibration = 'file://' + os.path.join(
        get_package_share_directory('my_robot_vision'),
        'camera_info', 'camera_calibration.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='5600'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('camera_info_url', default_value=default_calibration),
        DeclareLaunchArgument('frame_id', default_value='camera_optical_frame'),
        DeclareLaunchArgument(
            'rotate_180', default_value='false',
            description='Fallback CPU rotation when the Pi stream is not already rotated.',
        ),
        Node(
            package='my_robot_vision',
            executable='rtp_camera_publisher',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'frame_id': LaunchConfiguration('frame_id'),
                'rotate_180': LaunchConfiguration('rotate_180'),
            }],
        ),
    ])
