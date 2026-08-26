"""Launch the Raspberry Pi Camera Module 2 publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='10.0'),
        DeclareLaunchArgument(
            'camera_info_url', default_value='',
            description='file:///absolute/path/to/ROS_camera_calibration.yaml',
        ),
        Node(
            package='my_robot_vision',
            executable='pi_camera_publisher',
            output='screen',
            parameters=[{
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }],
        ),
    ])
