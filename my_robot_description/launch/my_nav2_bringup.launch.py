import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )

    # ใช้ navigation_launch.py ของ nav2_bringup แต่ไม่เรียก docking
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    return LaunchDescription([
        declare_map_arg,
        declare_use_sim_time,
        declare_params_file,
        navigation_launch
    ])
