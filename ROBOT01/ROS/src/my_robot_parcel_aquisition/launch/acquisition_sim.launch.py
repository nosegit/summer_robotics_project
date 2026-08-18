import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package, launch_file, arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=(arguments or {}).items())


def generate_launch_description():
    return LaunchDescription([
        include('my_robot_sim', 'gazebo.launch.py', {
            'headless': 'true', 'robot_x': '-5.55', 'robot_y': '-3.25',
            'robot_yaw': '1.5708',
        }),
        include('my_robot_vision', 'aruco_pose_estimator.launch.py', {
            'marker_size': '0.09', 'dictionary_name': 'DICT_4X4_250',
            'cube_size': '0.10',
        }),
        include('my_robot_parcel_aquisition', 'acquisition.launch.py', {'use_sim_time': 'true', 'cmd_vel_topic': '/cmd_vel'}),
    ])
