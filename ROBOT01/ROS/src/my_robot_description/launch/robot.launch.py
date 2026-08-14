import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # <-- Import ParameterValue

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mecanum_robot.urdf.xacro')

    use_sim = LaunchConfiguration('use_sim')
    publish_default_joints = LaunchConfiguration('publish_default_joints')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Toggle simulation (Gazebo plugins) vs real hardware (ros2_control)'
    )

    declare_publish_default_joints = DeclareLaunchArgument(
        'publish_default_joints',
        default_value='true',
        description='Launch non-GUI joint_state_publisher to broadcast default 0.0 joint positions'
    )

    # Process Xacro file and explicitly force string type
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=', use_sim]),
        value_type=str
    )

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim
        }]
    )

    # 2. Joint State Publisher (Headless)
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim
        }],
        condition=IfCondition(publish_default_joints)
    )

    return LaunchDescription([
        declare_use_sim,
        declare_publish_default_joints,
        node_robot_state_publisher,
        node_joint_state_publisher
    ])