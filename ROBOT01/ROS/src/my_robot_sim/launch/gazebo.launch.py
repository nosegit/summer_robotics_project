import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_sim_path = get_package_share_directory('my_robot_sim')
    pkg_description_path = get_package_share_directory('my_robot_description')

    world_file = os.path.join(pkg_sim_path, 'worlds', 'isa_map.sdf')
    xacro_file = os.path.join(pkg_description_path, 'urdf', 'mecanum_robot.urdf.xacro')

    headless = LaunchConfiguration('headless')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')

    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo Sim in server-only headless mode (-s --headless-rendering)'
    )
    declare_robot_x_arg = DeclareLaunchArgument('robot_x', default_value='-1.0')
    declare_robot_y_arg = DeclareLaunchArgument('robot_y', default_value='-1.0')
    declare_robot_yaw_arg = DeclareLaunchArgument('robot_yaw', default_value='0.0')

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

    # 2. Launch Gazebo directly via ExecuteProcess to allow timeout variables
    # Headless Mode (headless=true)
    gazebo_sim_headless = ExecuteProcess(
        condition=IfCondition(headless),
        cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world_file],
        output='screen',
        sigterm_timeout='15.0'  # Gives Gazebo 15 seconds to shut down gracefully before sending SIGTERM
    )

    # GUI Mode (headless=false)
    gazebo_sim_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        sigterm_timeout='15.0'  # Gives Gazebo 15 seconds to shut down gracefully before sending SIGTERM
    )

    # 3. Spawn Robot in Gazebo World
    node_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mecanum_robot',
            '-x', robot_x,
            '-y', robot_y,
            '-z', '0.05',
            '-Y', robot_yaw
        ]
    )

    # 4. ROS-Gazebo Topic Bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odometry/wheel@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu_sensor_L_broadcaster/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/imu_sensor_R_broadcaster/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
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
        declare_robot_x_arg,
        declare_robot_y_arg,
        declare_robot_yaw_arg,
        node_robot_state_publisher,
        gazebo_sim_headless,
        gazebo_sim_gui,
        node_spawn_robot,
        node_ros_gz_bridge
    ])