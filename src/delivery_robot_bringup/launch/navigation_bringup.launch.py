import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    bringup_dir = get_package_share_directory('delivery_robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'office_map.yaml'),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # 1. Include the official Nav2 launch file
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': 'False'}.items()
    )

    # 2. Launch your custom Mission Coordinator node
    mission_coordinator_cmd = Node(
        package='delivery_robot_core',
        executable='mission_coordinator',
        name='mission_coordinator',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(mission_coordinator_cmd)

    return ld