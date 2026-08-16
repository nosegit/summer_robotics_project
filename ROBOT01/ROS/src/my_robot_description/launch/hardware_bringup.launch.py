import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('my_robot_description')

    xacro_file = os.path.join(
        description_share, 'urdf', 'mecanum_robot.urdf.xacro')
    controller_config = os.path.join(
        description_share, 'config', 'mecanum_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false']),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        remappings=[
            ('/mecanum_drive_controller/odometry', '/odom'),
            ('/mecanum_drive_controller/tf_odometry', '/tf'),
        ],
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False},
            controller_config,
        ],
    )

    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager',
            ],
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

    # teleop_twist_keyboard publishes geometry_msgs/Twist on /cmd_vel, while
    # the Jazzy mecanum controller consumes TwistStamped on its reference topic.
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/mecanum_drive_controller/reference'),
        ],
    )

    start_after_controller_manager = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=controller_spawners,
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        start_after_controller_manager,
        twist_stamper,
    ])
