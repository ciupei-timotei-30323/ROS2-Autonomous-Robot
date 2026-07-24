import os
from launch import LaunchDescription
from launch_ros.actions import Node
# Import the package locator utility
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Dynamically find where the description package is installed
    description_dir = get_package_share_directory('delivery_robot_bringup')
    
    # Combine the paths cleanly using os.path.join
    urdf_file = os.path.join(description_dir, 'urdf', 'robot.urdf')
    ekf_config_file = os.path.join(description_dir, 'config', 'ekf.yaml')
    
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

        # DDSM115 Four Wheel Kinematics Node (cmd_vel -> rpm)
        Node(
            package='ddsm115_controller',
            executable='four_wheels_robot',
            name='four_wheels_robot_node',
            output='screen',
            parameters=[{
                'pub_tf': False,
                'yaw_odom_scale': 0.75,  # Start with 0.75, adjust if it turns too much (<1.0) or too little (>1.0) in rviz
                'yaw_mcu_scale': 0.0174533
            }],
            remappings=[
                ('/odom', '/old_odom')
            ]
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
                {'frame_id': 'laser_frame'},     
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
        ),

        # Yaw to Pose Translator Node for EKF
        Node(
            package='delivery_robot_core',
            executable='yaw_to_pose_node',
            name='yaw_to_pose_node',
            output='screen',
            parameters=[{
                'yaw_mcu_scale': 0.0174533
            }]
        ),

        # Robot Localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file],
            remappings=[('/odometry/filtered', '/odom')]
        )
    ])