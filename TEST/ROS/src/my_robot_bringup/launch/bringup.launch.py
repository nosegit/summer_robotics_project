import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate directories
    pkg_share = get_package_share_directory('my_robot_bringup')
    
    # 2. Xacro / Robot State Publisher
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )

    # # 3. Headless Gazebo Launch (Forces server-only headless mode)
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    #     ]),
    #     launch_arguments={'gz_args': '-r -s empty.sdf'}.items()
    # )

    # 3. Headless Gazebo Launch (Using your local .sdf file)
    world_file = os.path.join(pkg_share, 'worlds', 'simple_room.sdf')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        # Point gz_args to your local file!
        # launch_arguments={'gz_args': ['-r -s --headless-rendering ', world_file]}.items()
        launch_arguments={'gz_args': ['-r -s ', world_file]}.items()
        # launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    # 4. Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'simple_room',
            '-name', 'mecanum_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # 5. ROS-Gazebo Parameter Bridge (The Network Translator)
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )

    # 6. Include Nav2 Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'slam': 'True', 
            # 'slam': 'False', 
            'use_sim_time': 'True',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml')
        }.items()
    )

    # 7. ADD THIS: Explicitly launch the Collision Monitor
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        parameter_bridge,
        nav2_launch,
        collision_monitor_node # <--- Don't forget to add it to the return list!
    ])


