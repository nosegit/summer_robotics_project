import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
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

    controller_types = {
        'joint_state_broadcaster':
            'joint_state_broadcaster/JointStateBroadcaster',
        'imu_sensor_L_broadcaster':
            'imu_sensor_broadcaster/IMUSensorBroadcaster',
        'imu_sensor_R_broadcaster':
            'imu_sensor_broadcaster/IMUSensorBroadcaster',
        'ir_sensor_1_broadcaster':
            'range_sensor_broadcaster/RangeSensorBroadcaster',
        'ir_sensor_2_broadcaster':
            'range_sensor_broadcaster/RangeSensorBroadcaster',
        'ir_sensor_3_broadcaster':
            'range_sensor_broadcaster/RangeSensorBroadcaster',
        'ir_sensor_4_broadcaster':
            'range_sensor_broadcaster/RangeSensorBroadcaster',
        'servo_L':
            'position_controllers/JointGroupPositionController',
        'servo_R':
            'position_controllers/JointGroupPositionController',
        'mecanum_drive_controller':
            'mecanum_drive_controller/MecanumDriveController',
    }

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
        # Do not pass controller_config here. ros2_control on Jazzy drops
        # parameter-file paths containing "robot_description", leaving a bare
        # --params-file argument. The spawners load that file explicitly below.
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
            'update_rate': 50,
            **{
                f'{name}.type': controller_type
                for name, controller_type in controller_types.items()
            },
        }],
    )

    # A single spawner loads controllers in order. Starting one spawner per
    # controller concurrently can make controller_manager load calls time out
    # while the two serial hardware components are coming online.
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            *controller_types.keys(),
            '--controller-manager', '/controller_manager',
            '--param-file', controller_config,
            '--service-call-timeout', '30.0',
            '--switch-timeout', '30.0',
        ],
    )

    # Move both gripper servos to a known neutral angle as soon as their
    # controllers are active. The hardware interface and firmware use degrees.
    default_servo_L_angle = 110.0
    default_servo_R_angle = 80.0

    def initialize_servos_after_spawn(event, _context):
        if event.returncode != 0:
            return [LogInfo(
                msg='ERROR: Controller spawning failed; servo initialization skipped.'
            )]

        return [
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/servo_L/commands',
                    'std_msgs/msg/Float64MultiArray',
                    f'{{data: [{default_servo_L_angle}]}}',
                ],
                output='screen',
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/servo_R/commands',
                    'std_msgs/msg/Float64MultiArray',
                    f'{{data: [{default_servo_R_angle}]}}',
                ],
                output='screen',
            ),
        ]

    initialize_servos = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=initialize_servos_after_spawn,
        )
    )

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
            on_start=[controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        start_after_controller_manager,
        initialize_servos,
        twist_stamper,
    ])
