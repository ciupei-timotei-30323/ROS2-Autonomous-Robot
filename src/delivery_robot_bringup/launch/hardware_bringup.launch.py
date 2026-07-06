import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Path to your URDF file
    # Make sure you update 'delivery_robot_description' to your actual package path structure if different
    urdf_file = '/root/delivery_robot_ws/src/delivery_robot_description/urdf/delivery_robot.urdf'
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        
        # 2. Robot State Publisher (Broadcasts the TF tree based on your URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 3. Your Custom MCU Interface Node
        Node(
            package='delivery_robot_mcu',
            executable='mcu_interface',
            name='mcu_interface_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 115200}
            ]
        ),

        # 4. RPLidar A1 Driver Node
        # Requires: sudo apt install ros-jazzy-rplidar-ros
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB1'}, # Ensure this differs from your MCU port
                {'serial_baudrate': 115200},
                {'frame_id': 'laser_frame'},     # Must match the link name in the URDF
                {'inverted': False},
                {'angle_compensate': True}
            ]
        ),

        # 5. Rosbridge Server (WebSockets for Jetson & Face Pi)
        # Requires: sudo apt install ros-jazzy-rosbridge-server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}]
        )
    ])