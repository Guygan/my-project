import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    LogInfo,
    DeclareLaunchArgument,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_base_path = '/home/guygan/ros2_ws/src/my_robot_description'
    urdf_path = os.path.join(pkg_base_path, 'urdf', 'my_robot.urdf.xacro')
    bridge_config_path = os.path.join(pkg_base_path, 'config', 'gz_bridge.yaml')
    rviz_config_path = os.path.join(pkg_base_path, 'rviz', 'slam_view.rviz')
    ekf_config_path = os.path.join(pkg_base_path, 'config', 'ekf.yaml')
    slam_params_path = os.path.join(pkg_base_path, 'config', 'slam_params.yaml')

    # === Arguments ===
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='ใช้เวลาจำลองหรือไม่'
    )

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str
    )

    # === Gazebo ===
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # === Nodes ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content
        }]
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot'],
        output='screen'
    )

    gz_ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    frame_fixer_node = Node(
        package='my_robot_description',
        executable='frame_fixer',
        name='scan_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    laser_to_sonar_node = Node(
        package='my_robot_description',
        executable='laser_to_sonar',
        name='laser_to_sonar_node',
        output='screen',
        remappings=[('/scan', '/scan_corrected')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    autonomous_explorer_node = Node(
        package='my_robot_description',
        executable='autonomous_explorer',
        name='smart_autonomous_explorer',
        output='screen',
        remappings=[('/scan', '/scan_corrected')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params_path
        }.items()
    )

    # === Delay handler ===
    delay_nodes_after_spawn_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                LogInfo(msg='✅ Robot spawned. Launching bridge, EKF, and other nodes...'),
                gz_ros_bridge_node,
                robot_localization_node,
                frame_fixer_node,
                rviz_node,
                laser_to_sonar_node,
                TimerAction(
                    period=6.0,
                    actions=[
                        LogInfo(msg='✅ Delay complete. Launching SLAM Toolbox & Autonomous Explorer...'),
                        start_slam_toolbox,
                        autonomous_explorer_node
                    ]
                )
            ]
        )
    )

    # === Launch Description ===
    return LaunchDescription([
        use_sim_time_arg,
        start_gazebo_cmd,
        robot_state_publisher_node,
        spawn_robot_node,
        delay_nodes_after_spawn_handler,
    ])
