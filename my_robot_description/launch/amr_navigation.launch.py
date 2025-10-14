# auto_slam.launch.py
# Fully autonomous SLAM + Exploration + Auto map/world save (ROS 2 Jazzy + Gazebo Harmonic)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_my_robot_description = get_package_share_directory('my_robot_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # --- Paths ---
    world_file = os.path.join(pkg_my_robot_description, 'worlds', 'amr_world.sdf')
    map_file = os.path.join(pkg_my_robot_description, 'maps', 'amr_map.yaml')
    slam_params = os.path.join(pkg_my_robot_description, 'config', 'slam_params.yaml')
    nav_params = os.path.join(pkg_my_robot_description, 'config', 'amr_nav_params.yaml')
    rviz_config = os.path.join(pkg_my_robot_description, 'rviz', 'my_nav2_view.rviz')

    # --- Launch Args ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=nav_params)
    rviz_file = LaunchConfiguration('rviz_config', default=rviz_config)

    # --- Gazebo ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_my_robot_description, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'world': ''}.items()
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- SLAM Toolbox ---
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': slam_params}.items()
    )

    # --- Nav2 (No docking server) ---
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_rviz': 'false',
            'launch_docking_server': 'false'
        }.items()
    )

    # --- Autonomous Exploration ---
    explorer_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'costmap_topic': '/global_costmap/costmap',
            'planner_frequency': 0.5
        }]
    )

    # --- Auto-save map & world when RViz closes ---
    save_map_cmd = ExecuteProcess(
        cmd=['bash', '-c', f'sleep 3 && ros2 run nav2_map_server map_saver_cli -f {map_file.replace(".yaml", "")}'],
        output='screen'
    )

    save_world_cmd = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/default/save',
            '--reqtype', 'gz.msgs.StringMsg', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000', '--req', f'data: "{world_file}"'
        ],
        output='screen'
    )

    save_on_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[
                LogInfo(msg='🧭 RViz closed — Saving map and Gazebo world...'),
                save_map_cmd,
                save_world_cmd
            ]
        )
    )

    # --- Delay startup for stability ---
    delayed_slam_nav = TimerAction(period=5.0, actions=[slam_toolbox, nav2_bringup])
    delayed_explorer = TimerAction(period=8.0, actions=[explorer_node])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=nav_params),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        LogInfo(msg='🚀 Launching Autonomous SLAM & Exploration Mode...'),
        gazebo_launch,
        rviz_node,
        delayed_slam_nav,
        delayed_explorer,
        save_on_exit_handler
    ])
