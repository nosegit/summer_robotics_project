import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('my_robot_parcel_aquisition'), 'config', 'acquisition.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'config_file', default_value=config,
            description='Parcel acquisition parameter YAML.',
        ),
        Node(
            package='my_robot_parcel_aquisition', executable='acquisition_main',
            output='screen', parameters=[LaunchConfiguration('config_file'), {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }],
        ),
    ])
