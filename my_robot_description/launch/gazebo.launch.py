# my_robot_description/launch/gazebo.launch.py (Final Corrected Version)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # Define paths
    urdf_path = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf.xacro')
    bridge_config_path = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')
    default_world_path = os.path.join(pkg_dir, 'worlds', 'test_world.sdf')

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_world_arg = DeclareLaunchArgument('world', default_value=default_world_path)

    # Use the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # Robot State Publisher
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}]
    )

    # Gazebo Sim Server
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world}.items()
    )

    # Spawn Robot Node
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_my_robot',
        arguments=['-topic', 'robot_description', '-name', 'my_robot'],
        output='screen'
    )

    # Gazebo-ROS Bridge
    gz_ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_world_arg,
        gz_sim_server,
        robot_state_publisher,
        spawn_robot_node,
        gz_ros_bridge_node,
    ])