# my_robot_description/launch/robot_localization.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run the full robot simulation stack, including:
    - Gazebo with the specified world.
    - Robot localization using an EKF filter.
    - The Nav2 navigation stack.
    - RViz for visualization.
    - Keyboard teleoperation in a separate terminal.
    """
    # --- 1. Package and File Path Definitions ---
    pkg_dir = get_package_share_directory('my_robot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    world_path = os.path.join(pkg_dir, 'worlds', 'test_world.sdf')
    map_path = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'my_nav2_view.rviz')

    # --- 2. Launch Configuration Variables ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # --- 3. Node and Action Definitions ---

    # Action to launch Gazebo and spawn the robot
    start_gazebo_and_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'use_sim_time': use_sim_time}.items()
    )

    # Node for the frame fixer to correct laser scan data
    frame_fixer = Node(
        package='my_robot_description',
        executable='frame_fixer',
        name='scan_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Node for EKF from robot_localization to fuse sensor data
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    # Action to launch the Nav2 stack
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # Node to launch RViz with a pre-configured view
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/scan', '/scan_corrected')],
        output='screen'
    )

    # Node to launch keyboard teleop in a new terminal window
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    # --- 4. Assemble Launch Description ---
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='Full path to the world file to load'),
        DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description='Full path to the map file for Nav2'),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_path,
            description='Full path to the Nav2 parameters file'),

        # Add nodes and actions to the launch description
        start_gazebo_and_robot,
        frame_fixer,
        robot_localization,
        start_nav2,
        start_rviz,
        keyboard_teleop
    ])