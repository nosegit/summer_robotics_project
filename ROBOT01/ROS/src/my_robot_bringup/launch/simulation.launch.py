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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigation_share = get_package_share_directory('my_robot_navigation')
    simulation_share = get_package_share_directory('my_robot_sim')
    localization_share = get_package_share_directory('my_robot_localization')
    nav2_share = get_package_share_directory('nav2_bringup')

    mode = LaunchConfiguration('mode')
    headless = LaunchConfiguration('headless')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    parcel_acquisition = LaunchConfiguration('parcel_acquisition')
    debug_image = LaunchConfiguration('debug_image')
    nav2_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'cmd_vel_out_topic': '/cmd_vel_navigation'},
        convert_types=True,
    )

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
            'params_file': nav2_params,
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items(),
    )

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_vision'),
                'launch',
                'aruco_pose_estimator.launch.py',
            )
        ),
        condition=IfCondition(parcel_acquisition),
        launch_arguments={
            'marker_size': '0.09',
            'dictionary_name': 'DICT_4X4_250',
            'cube_size': '0.10',
            'position_filter_enabled': 'true',
            'position_filter_alpha': '1.0',
            'position_outlier_threshold': '0.1',
            'debug_image': debug_image,
        }.items(),
    )

    acquisition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_parcel_aquisition'),
                'launch',
                'acquisition.launch.py',
            )
        ),
        condition=IfCondition(parcel_acquisition),
        launch_arguments={
            'use_sim_time': 'true',
            'cmd_vel_topic': '/cmd_vel_acquisition',
            'config_file': os.path.join(
                get_package_share_directory('my_robot_parcel_aquisition'),
                'config',
                'acquisition_sim.yaml',
            ),
        }.items(),
    )

    cmd_vel_arbiter = Node(
        package='my_robot_bringup',
        executable='cmd_vel_arbiter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        # Nav2's isolated component container can need longer than the launch
        # default of five seconds to unload all components during shutdown.
        SetLaunchConfiguration('sigterm_timeout', '15.0'),
        DeclareLaunchArgument(
            'mode', default_value='localization',
            description='Nav2 mode: mapping (SLAM) or localization (saved map).',
        ),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument(
            'parcel_acquisition', default_value='true',
            description='Start parcel vision and acquisition action servers.',
        ),
        DeclareLaunchArgument(
            'debug_image', default_value='true',
            description='Publish parcel vision debug images.',
        ),
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
        vision,
        acquisition,
        cmd_vel_arbiter,
    ])
