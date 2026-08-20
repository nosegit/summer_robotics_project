"""Start the robot hardware, lidar, Nav2, vision, and parcel acquisition.

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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    description_share = get_package_share_directory('my_robot_description')
    vision_share = get_package_share_directory('my_robot_vision')
    navigation_share = get_package_share_directory('my_robot_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    rplidar_share = get_package_share_directory('rplidar_ros')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    parcel_acquisition = LaunchConfiguration('parcel_acquisition')
    debug_image = LaunchConfiguration('debug_image')
    camera_info_url = LaunchConfiguration('camera_info_url')
    nav2_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'cmd_vel_out_topic': '/cmd_vel_navigation'},
        convert_types=True,
    )

    hardware_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_share, 'launch', 'hardware_bringup.launch.py'
            )
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
            'params_file': nav2_params,
            'use_sim_time': 'False',
            'autostart': 'True',
        }.items(),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_vision'),
                'launch',
                # 'pi_camera.launch.py',
                'rtp_camera.launch.py',
            )
        ),
        condition=IfCondition(parcel_acquisition),
        launch_arguments={
            'camera_info_url': camera_info_url,
            # Do not inherit the lidar include's frame_id=laser_frame launch
            # configuration. Camera detections are expressed in optical axes.
            'frame_id': 'camera_optical_frame',
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
            'marker_size': '0.07',
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
            'use_sim_time': 'false',
            'cmd_vel_topic': '/cmd_vel_acquisition',
        }.items(),
    )
    cmd_vel_arbiter = Node(
        package='my_robot_bringup',
        executable='cmd_vel_arbiter',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        # Nav2's isolated component container can need longer than the launch
        # default of five seconds to unload all components during shutdown.
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
        DeclareLaunchArgument(
            'parcel_acquisition', default_value='true',
            description='Start the Pi camera, parcel vision, and acquisition actions.',
        ),
        DeclareLaunchArgument(
            'debug_image', default_value='false',
            description='Publish parcel vision debug images.',
        ),
        DeclareLaunchArgument(
            'camera_info_url', default_value='file://' + os.path.join(
                vision_share, 'camera_info', 'camera_calibration.yaml'
            ),
            description='file:/// URL of the Pi camera calibration YAML.',
        ),
        hardware_description,
        lidar,
        nav2,
        camera,
        vision,
        acquisition,
        cmd_vel_arbiter,
    ])
