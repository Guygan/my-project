# In file: my_robot_description/launch/gazebo.launch.py
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
    urdf_path = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf.xacro')
    bridge_config_path = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')

    declare_world_arg = DeclareLaunchArgument('world', description='Full path to the world file to load.')
    declare_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    robot_description_content = Command(['xacro ', urdf_path])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['gz_args', LaunchConfiguration('world')]}.items() 
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot']
    )

    gz_ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}]
    )

    return LaunchDescription([
        declare_world_arg,
        declare_use_sim_time_arg,
        robot_state_publisher_node,
        gz_sim_server,
        spawn_robot_node,
        gz_ros_bridge_node,
    ])