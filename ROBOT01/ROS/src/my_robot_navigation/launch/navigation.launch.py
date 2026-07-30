import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_nav = get_package_share_directory('my_robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'))
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_nav, 'maps', 'simple_room.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=map_yaml_file),
        DeclareLaunchArgument('params_file', default_value=params_file),
        nav2_bringup
    ])