"""Hardware bringup template: ros2_control, RPLIDAR A1, and Nav2.

Verify serial devices, controller calibration, transforms, and safety limits before use.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('my_robot_description')
    navigation_share = get_package_share_directory('my_robot_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    rplidar_share = get_package_share_directory('rplidar_ros')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')

    xacro_file = os.path.join(description_share, 'urdf', 'mecanum_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false']), value_type=str
    )
    bringup_share = get_package_share_directory('my_robot_bringup')
    controller_config = os.path.join(
        bringup_share, 'config', 'mecanum_controllers.yaml'
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_sim': 'false',
            'publish_default_joints': 'false',
        }.items(),
    )

    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[{'robot_description': robot_description}, controller_config],
    )
    controller_spawners = [
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[controller, '--controller-manager', '/controller_manager'],
        )
        for controller in (
            'joint_state_broadcaster',
            'imu_sensor_1_broadcaster',
            'imu_sensor_2_broadcaster',
            'ir_sensor_1_broadcaster',
            'ir_sensor_2_broadcaster',
            'ir_sensor_3_broadcaster',
            'ir_sensor_4_broadcaster',
            'mecanum_drive_controller',
        )
    ]
    start_controllers = RegisterEventHandler(
        OnProcessStart(target_action=controller_manager, on_start=controller_spawners)
    )

    twist_stamper = Node(
        package='twist_stamper', executable='twist_stamper', name='twist_stamper',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/mecanum_drive_controller/reference'),
        ],
        output='screen',
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='localization',
            description='Nav2 mode: mapping (SLAM) or localization (saved map).',
        ),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(navigation_share, 'maps', 'simple_room.yaml'),
        ),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(navigation_share, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB0',
            description='RPLIDAR A1 serial device; verify this on the robot.',
        ),
        DeclareLaunchArgument('lidar_baudrate', default_value='115200'),
        description,
        controller_manager,
        start_controllers,
        twist_stamper,
        lidar,
        nav2,
    ])
