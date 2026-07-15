import os
from launch import LaunchDescription
from launch_ros.actions import Node
# Import the package locator utility
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Dynamically find where the description package is installed
    description_dir = get_package_share_directory('delivery_robot_bringup')
    
    # Combine the paths cleanly using os.path.join
    urdf_file = os.path.join(description_dir, 'urdf', 'delivery_robot.urdf')
    
    from launch.substitutions import Command

    return LaunchDescription([
        
        # Robot State Publisher (Broadcasts the TF tree based on your URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),

        # DDSM115 Motor Controller (RS485 Communication)
        Node(
            package='ddsm115_controller',
            executable='velocity_control',
            name='velocity_control_node',
            output='screen'
        ),

        # DDSM115 Four Wheel Kinematics Node (cmd_vel -> rpm -> odom)
        Node(
            package='ddsm115_controller',
            executable='four_wheels_robot',
            name='four_wheels_robot_node',
            output='screen'
        ),

        # RPLidar A1 Driver Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'}, # Changed to ttyUSB0 since it's the only device connected
                {'serial_baudrate': 115200},
                {'frame_id': 'lidar_link'},     
                {'inverted': False},
                {'angle_compensate': True}
            ]
        ),


        # TCP Bridge Node for minimalist custom protocol
        Node(
            package='delivery_robot_core',
            executable='tcp_bridge_node',
            name='tcp_bridge_node',
            output='screen'
        )
    ])