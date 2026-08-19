"""Start Gazebo, the robot, and a complete Nav2 stack.

Use ``mode:=mapping`` for SLAM or ``mode:=localization`` to navigate a saved map.
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


def generate_launch_description():
    navigation_share = get_package_share_directory('my_robot_navigation')
    simulation_share = get_package_share_directory('my_robot_sim')
    localization_share = get_package_share_directory('my_robot_localization')
    nav2_share = get_package_share_directory('nav2_bringup')

    mode = LaunchConfiguration('mode')
    headless = LaunchConfiguration('headless')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'headless': headless}.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            # Nav2 starts slam_toolbox and omits AMCL when this is True.
            'slam': PythonExpression(["'True' if '", mode, "' == 'mapping' else 'False'"]),
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items(),
    )

    return LaunchDescription([
        SetLaunchConfiguration('sigterm_timeout', '15.0'),
        DeclareLaunchArgument(
            'mode', default_value='localization',
            description='Nav2 mode: mapping (SLAM) or localization (saved map).',
        ),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument(
            'map',
            # default_value=os.path.join(navigation_share, 'maps', 'simple_room.yaml'),
            default_value=os.path.join(navigation_share, 'maps', 'isa_map.yaml'),
            description='Map YAML used when mode:=localization.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(navigation_share, 'config', 'nav2_params_sim.yaml'),
        ),
        gazebo,
        localization,
        nav2,
    ])
