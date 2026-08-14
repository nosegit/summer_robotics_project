import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_sim_path = get_package_share_directory('my_robot_sim')
    pkg_description_path = get_package_share_directory('my_robot_description')

    # world_file = os.path.join(pkg_sim_path, 'worlds', 'simple_room.sdf')
    world_file = os.path.join(pkg_sim_path, 'worlds', 'isa_map.sdf')
    xacro_file = os.path.join(pkg_description_path, 'urdf', 'mecanum_robot.urdf.xacro')

    headless = LaunchConfiguration('headless')

    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo Sim in server-only headless mode (-s --headless-rendering)'
    )

    # Process Xacro with simulation plugins enabled
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=true']), value_type=str
    )

    # 1. Start Robot State Publisher with use_sim_time enabled
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 2. Include Gazebo Sim Launch (Headless Server vs GUI Server)
    # When headless=true -> runs: gz sim -r -s --headless-rendering <world>
    # When headless=false -> runs: gz sim -r <world>
    gz_args = PythonExpression([
        "'-r -s --headless-rendering ", world_file, "' if '", headless, "' == 'true' else '-r ", world_file, "'"
    ])

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # 3. Spawn Robot in Gazebo World
    node_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mecanum_robot',
            '-x', '-1.0',
            '-y', '-1.0',
            '-z', '0.05'
        ]
    )

    # 4. ROS-Gazebo Topic Bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ]
    )

    return LaunchDescription([
        declare_headless_arg,
        node_robot_state_publisher,
        gazebo_sim,
        node_spawn_robot,
        node_ros_gz_bridge
    ])
