import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'arduino_hardware_interface'
    
    # Point to the test folder where these files are stored
    test_dir = os.path.join(get_package_share_directory(pkg_name), 'test_file')
    
    xacro_file = os.path.join(test_dir, 'mecanum_hardware.urdf.xacro')
    controller_config = os.path.join(test_dir, 'mecanum_controllers.yaml')
    
    # Wrap the Command in ParameterValue and set value_type=str
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false']),
        value_type=str
    )

    # 1. Robot State Publisher (publishes the TF tree)
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_config],
        emulate_tty=True,
        output='screen'
    )

    # 3. Twist Stamper Node
    # Converts geometry_msgs/Twist on /cmd_vel to geometry_msgs/TwistStamped on /mecanum_drive_controller/reference
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/mecanum_drive_controller/reference'),
        ],
        output='screen'
    )

    # 4. Define the controllers you want to activate
    active_controllers = [
        "joint_state_broadcaster",
        "imu_sensor_1_broadcaster",
        "imu_sensor_2_broadcaster",
        "ir_sensor_1_broadcaster",
        "ir_sensor_2_broadcaster",
        "ir_sensor_3_broadcaster",
        "ir_sensor_4_broadcaster",
        "mecanum_drive_controller"
    ]

    spawner_nodes = []
    for controller in active_controllers:
        spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
            )
        )

    # 5. Ensure spawners and twist_stamper wait for the controller_manager to boot
    delay_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=spawner_nodes + [twist_stamper_node],
        )
    )

    return LaunchDescription([
        robot_state_pub_node,
        controller_manager,
        delay_spawners
    ])