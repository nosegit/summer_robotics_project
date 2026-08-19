import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, launch_file, arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=(arguments or {}).items())


def generate_launch_description():
    acquisition_sim_config = os.path.join(
        get_package_share_directory('my_robot_parcel_aquisition'),
        'config',
        'acquisition_sim.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('debug_image', default_value='true'),
        include('my_robot_sim', 'gazebo.launch.py', {
            'robot_x': '-5.55', 'robot_y': '-3.25',
            'robot_yaw': '1.5708',
        }),
        include('my_robot_vision', 'aruco_pose_estimator.launch.py', {
            'marker_size': '0.09', 'dictionary_name': 'DICT_4X4_250',
            'cube_size': '0.10', 
            'position_filter_enabled': 'true', 'position_filter_alpha': '1.0', 'position_outlier_threshold': '0.1',
            # 'debug_image': 'true',
            'debug_image': LaunchConfiguration('debug_image'),

        }),
        include('my_robot_parcel_aquisition', 'acquisition.launch.py', {
            'use_sim_time': 'true',
            'cmd_vel_topic': '/cmd_vel',
            'config_file': acquisition_sim_config,
        }),
    ])
