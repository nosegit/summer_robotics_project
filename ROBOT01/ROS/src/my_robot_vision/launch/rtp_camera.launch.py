"""Receive the host Pi camera RTP stream and publish standard ROS images."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='5600'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('frame_id', default_value='camera_optical_frame'),
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
            }],
        ),
    ])
