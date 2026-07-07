import os
from launch import LaunchDescription
from launch_ros.actions import Node
# Import the package locator utility
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Dynamically find where the description package is installed
    description_dir = get_package_share_directory('delivery_robot_description')
    
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

        # Your Custom MCU Interface Node (COMMENTED OUT since only LiDAR is connected)
        # Node(
        #     package='delivery_robot_mcu',
        #     executable='mcu_interface',
        #     name='mcu_interface_node',
        #     output='screen',
        #     parameters=[
        #         {'port': '/dev/ttyUSB0'},
        #         {'baudrate': 115200}
        #     ]
        # ),

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

        # Rosbridge Server (WebSockets for Jetson & Face Pi)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}]
        ),

        # TCP Bridge Node for minimalist custom protocol
        Node(
            package='delivery_robot_core',
            executable='tcp_bridge_node',
            name='tcp_bridge_node',
            output='screen'
        )
    ])