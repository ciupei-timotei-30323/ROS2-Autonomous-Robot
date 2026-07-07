import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Find your bringup package
    bringup_dir = get_package_share_directory('delivery_robot_bringup')
    
    # Path to your YAML file
    slam_config_file = os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        Node(
            parameters=[slam_config_file, {'use_sim_time': False}],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    ])