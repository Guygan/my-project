# my_robot_description/launch/robot_localization.launch.py (Corrected Version)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
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

    start_gazebo_and_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'use_sim_time': use_sim_time}.items()
    )

    frame_fixer = Node(
        package='my_robot_description',
        executable='frame_fixer',
        name='scan_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    velocity_smoother_node = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace='',  
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')]
    )

    collision_monitor_node = LifecycleNode(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        namespace='',
        output='screen',
        parameters=[params_file]
    )

    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_velocity_smoother': 'False'
        }.items()
    )

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/scan', '/scan_corrected')],
        output='screen'
    )

    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        # ✨ KEY FIX: Remap to the final cmd_vel topic
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    # --- 4. Assemble Launch Description ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_path),
        DeclareLaunchArgument('map', default_value=map_path),
        DeclareLaunchArgument('params_file', default_value=nav2_params_path),

        start_gazebo_and_robot,
        frame_fixer,
        robot_localization,
        start_nav2,
        start_rviz,
        velocity_smoother_node,   
        collision_monitor_node,  # ✨ KEY FIX: Enabled this node
        keyboard_teleop
    ])