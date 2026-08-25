"""Start the hardware description, RPLIDAR A1, and Nav2.

Verify serial devices, controller calibration, transforms, and safety limits before use.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('my_robot_description')
    navigation_share = get_package_share_directory('my_robot_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    rplidar_share = get_package_share_directory('rplidar_ros')
    gripper_share = get_package_share_directory('my_robot_gripper')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    red_led_gpio = LaunchConfiguration('red_led_gpio')
    green_led_gpio = LaunchConfiguration('green_led_gpio')

    hardware_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_share, 'launch', 'hardware_bringup.launch.py'
            )
        )
    )
    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripper_share, 'launch', 'gripper.launch.py')
        )
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_share, 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baudrate,
            'frame_id': 'laser_frame',
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': PythonExpression(["'True' if '", mode, "' == 'mapping' else 'False'"]),
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'False',
            'autostart': 'True',
        }.items(),
    )
    mission_led_controller = Node(
        package='my_robot_mission',
        executable='led_controller',
        output='screen',
        parameters=[{
            'red_gpio_pin': red_led_gpio,
            'green_gpio_pin': green_led_gpio,
        }],
    )

    return LaunchDescription([
        SetLaunchConfiguration('sigterm_timeout', '15.0'),
        DeclareLaunchArgument(
            'mode', default_value='localization',
            description='Nav2 mode: mapping (SLAM) or localization (saved map).',
        ),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(navigation_share, 'maps', 'isa_map.yaml'),
        ),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(navigation_share, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB0',
            description='RPLIDAR A1 serial device; verify this on the robot.',
        ),
        DeclareLaunchArgument('lidar_baudrate', default_value='115200'),
        DeclareLaunchArgument('red_led_gpio', default_value='17'),
        DeclareLaunchArgument('green_led_gpio', default_value='27'),
        hardware_description,
        gripper,
        lidar,
        nav2,
        mission_led_controller,
    ])
