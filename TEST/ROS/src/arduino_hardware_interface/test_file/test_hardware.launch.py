import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    # --- CONFIGURATION ---
    pkg_name = 'arduino_hardware_interface'
    
    # Point directly to your new 'test_file' folder
    test_dir = os.path.join(get_package_share_directory(pkg_name), 'test_file')
    
    urdf_file = os.path.join(test_dir, 'test_arduino.urdf')
    controller_config = os.path.join(test_dir, 'test_controllers.yaml')
    
    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # 1. Robot State Publisher
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
        output='screen'
    )

    # 3. Define all the controllers and broadcasters you want to activate
    active_controllers = [
        "joint_state_broadcaster",
        "imu_sensor_broadcaster",
        "ir_sensor_1_broadcaster",
        "ir_sensor_2_broadcaster",
        "wheel_velocity_controller",
        "servo_position_controller"
    ]

    # Generate a spawner node for each controller in the list
    spawner_nodes = []
    for controller in active_controllers:
        spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
            )
        )

    # 4. Ensure all spawners wait for the controller_manager to boot up first
    delay_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=spawner_nodes,
        )
    )

    return LaunchDescription([
        robot_state_pub_node,
        controller_manager,
        delay_spawners
    ])