import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_gripper'),
        'config',
        'gripper.yaml',
    )

    return LaunchDescription([
        Node(
            package='my_robot_gripper',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen',
            parameters=[config],
        ),
    ])
