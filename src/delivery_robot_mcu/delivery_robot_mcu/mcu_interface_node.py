#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import time

class McuInterfaceNode(Node):
    def __init__(self):
        super().__init__('mcu_interface_node')

        # --- Parameters ---
        # Allow the serial port and baud rate to be changed via launch files
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # --- Serial Connection ---
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Successfully connected to MCU on {port} at {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to MCU: {e}")
            # In a real scenario, you might want to retry, but we'll exit for simplicity if hardware is missing
            raise SystemExit

        # --- Publishers & Subscribers ---
        # Subscribes to velocity commands from Nav2 or manual teleop
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishes odometry for Nav2 and SLAM
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- State Variables for Odometry Integration ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # --- Timer for reading from MCU ---
        # Read at 50Hz (0.02 seconds)
        self.timer = self.create_timer(0.02, self.read_serial_data)

    def cmd_vel_callback(self, msg):
        """
        Translates a ROS 2 Twist message into a simple serial string for the MCU.
        Protocol: "CMD,<linear_x>,<angular_z>\n"
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        command_str = f"CMD,{linear_x:.3f},{angular_z:.3f}\n"
        
        try:
            self.serial_conn.write(command_str.encode('utf-8'))
        except serial.SerialException:
            self.get_logger().warn("Failed to write to MCU serial port.")

    def read_serial_data(self):
        """
        Reads encoder feedback from the MCU, integrates it to find position,
        and publishes the Odometry and TF.
        Protocol expected from MCU: "ODOM,<actual_linear_x>,<actual_angular_z>\n"
        """
        if not self.serial_conn.in_waiting:
            return

        try:
            # Read a line and decode it
            line = self.serial_conn.readline().decode('utf-8').strip()
            
            # Simple parsing
            if line.startswith("ODOM"):
                parts = line.split(',')
                if len(parts) == 3:
                    v_x = float(parts[1])
                    v_theta = float(parts[2])
                    
                    self.publish_odometry(v_x, v_theta)
                    
        except Exception as e:
            self.get_logger().warn(f"Error parsing serial data: {e}")

    def publish_odometry(self, v_x, v_theta):
        """
        Math to track where the robot is based on its speed, then broadcast it.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Integrate velocity to get position (Differential Drive Kinematics)
        delta_x = (v_x * math.cos(self.theta)) * dt
        delta_y = (v_x * math.sin(self.theta)) * dt
        delta_theta = v_theta * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.last_time = current_time

        # Create Quaternion from Euler angle (theta) for the messages
        q = self.euler_to_quaternion(0, 0, self.theta)

        # 1. Publish Transform (TF)
        # Nav2 and SLAM absolutely require this transform to know where the robot's base is relative to its starting point
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = v_theta
        
        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Helper function to convert Euler angles to quaternions.
        ROS 2 uses quaternions for all rotations.
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = McuInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()