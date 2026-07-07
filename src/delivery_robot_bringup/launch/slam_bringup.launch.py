import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Find your bringup package
    bringup_dir = get_package_share_directory('delivery_robot_bringup')
    
    # Path to your YAML file
    slam_config_file = os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml')

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={'slam_params_file': slam_config_file, 'use_sim_time': 'false'}.items()
        )
    ])