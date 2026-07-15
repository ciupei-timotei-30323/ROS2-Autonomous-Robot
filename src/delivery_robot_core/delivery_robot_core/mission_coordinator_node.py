#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import json
import os
from ament_index_python.packages import get_package_share_directory

class MissionCoordinatorNode(Node):
    def __init__(self):
        super().__init__('mission_coordinator_node')

        # --- FSM States ---
        self.state = "IDLE"  
        # States: IDLE, MOVING_FORWARD, MOVING_BACK, MOVING_LEFT, MOVING_RIGHT
        # AUTONOMOUS_NAVIGATION, STUCK

        # --- Parameters ---
        self.declare_parameter('safety_distance', 0.2)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.2)
        
        default_locations_file = os.path.join(
            get_package_share_directory('delivery_robot_core'),
            'config',
            'Locations.json'
        )
        self.declare_parameter('locations_file_path', default_locations_file)

        self.safety_distance = self.get_parameter('safety_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.locations_file_path = self.get_parameter('locations_file_path').value

        self.target_destination_name = ""
        self.target_coords = None
        self.stuck_timer = None
        self.goal_handle = None
        self.stuck_retries = 0

        # --- Publishers & Subscribers ---
        # 1. Listen for voice/tcp destination commands
        self.dest_sub = self.create_subscription(
            String,
            '/system/voice_destination',
            self.voice_dest_callback,
            10
        )

        # 2. Publish robot state back to external systems
        self.state_pub = self.create_publisher(String, '/system/robot_state', 10)
        
        # 3. Publish text alerts to be sent raw over TCP
        self.alert_pub = self.create_publisher(String, '/system/tcp_alert', 10)

        # 4. Publish cmd_vel for manual driving
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 5. Subscribe to /scan for obstacle detection in manual mode
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # --- Action Clients & Timers ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.control_timer = self.create_timer(0.1, self.control_loop) # 10Hz

        self.get_logger().info("Mission Coordinator initialized and waiting in IDLE state.")
        self.publish_state()

    def publish_state(self):
        """Broadcasts current FSM state to the external systems."""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
        self.get_logger().info(f"State changed to: {self.state}")
        
    def publish_alert(self, msg_str):
        """Broadcasts a string directly to TCP clients."""
        msg = String()
        msg.data = msg_str
        self.alert_pub.publish(msg)
        self.get_logger().info(f"Published TCP Alert: {msg_str}")

    def load_locations(self):
        filepath = self.locations_file_path
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load locations: {e}")
            return {}

    def cancel_nav_goal(self):
        if self.state == "AUTONOMOUS_NAVIGATION" and self.goal_handle:
            self.get_logger().info("Canceling active Nav2 goal.")
            self.goal_handle.cancel_goal_async()

    def voice_dest_callback(self, msg):
        """Triggered when the Jetson/TCP client sends a command string."""
        command = msg.data
        lower_command = command.lower()

        if command.startswith("GO "):
            destination = command[3:].strip()
            self.handle_go_command(destination)
            return

        if lower_command in ["stop", "automatic", "forward", "back", "left", "right"]:
            self.cancel_nav_goal()
            if self.stuck_timer:
                self.stuck_timer.cancel()
                self.stuck_timer = None
            self.stuck_retries = 0

        if lower_command == "stop":
            self.get_logger().info("STOP command received. Halting robot.")
            self.state = "IDLE"
            self.stop_robot()
        elif lower_command == "automatic":
            self.get_logger().info("Automatic mode triggered. Setting to IDLE.")
            self.state = "IDLE"
            self.stop_robot()
        elif lower_command == "forward":
            self.state = "MOVING_FORWARD"
        elif lower_command == "back":
            self.state = "MOVING_BACK"
        elif lower_command == "left":
            self.state = "MOVING_LEFT"
        elif lower_command == "right":
            self.state = "MOVING_RIGHT"
        else:
            self.get_logger().warn(f"Unknown voice command: {command}")
            return
            
        self.publish_state()

    def handle_go_command(self, destination):
        if self.state != "IDLE":
            self.get_logger().warn("Ignored GO command; robot is not IDLE.")
            return
            
        locations = self.load_locations()
        if destination not in locations:
            self.publish_alert("Location not known")
            return
            
        self.target_coords = locations[destination]
        self.target_destination_name = destination
        self.state = "AUTONOMOUS_NAVIGATION"
        self.stuck_retries = 0
        self.publish_state()
        self.send_nav_goal(destination, self.target_coords)

    def send_nav_goal(self, location_name, coords):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available!")
            self.state = "STUCK"
            self.publish_state()
            self.publish_alert("Robot is stuck")
            self.start_stuck_timer()
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = coords["x"]
        goal_msg.pose.pose.position.y = coords["y"]
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.z = math.sin(coords["theta"] / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(coords["theta"] / 2.0)

        self.get_logger().info(f"Sending goal to Nav2: {location_name}")
        
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the navigation goal.")
            self.state = "STUCK"
            self.publish_state()
            self.publish_alert("Robot is stuck")
            self.start_stuck_timer()
            return

        self.goal_handle = goal_handle
        self.get_logger().info("Nav2 accepted goal. Driving...")
        if self.stuck_timer:
            self.stuck_timer.cancel()
            self.stuck_timer = None
            
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # We might have been interrupted by a new manual command, ensure we're still in AUTONOMOUS_NAVIGATION
        if self.state != "AUTONOMOUS_NAVIGATION":
            return
            
        status = future.result().status
        
        if status == 4: # SUCCEEDED
            self.get_logger().info("Arrived at destination.")
            self.state = "IDLE"
            self.stuck_retries = 0
            self.publish_state()
            self.publish_alert("Robot arrived at destination")
        else:
            self.get_logger().error(f"Navigation failed with status code: {status}")
            self.state = "STUCK"
            self.publish_state()
            self.publish_alert("Robot is stuck")
            self.start_stuck_timer()

    def start_stuck_timer(self):
        if self.stuck_timer is None:
            self.stuck_timer = self.create_timer(5.0, self.stuck_recovery_callback)

    def stuck_recovery_callback(self):
        if self.state == "STUCK" and self.target_coords:
            if self.stuck_retries >= 4:
                self.get_logger().info("Max stuck retries reached, switching to IDLE to conserve battery.")
                self.state = "IDLE"
                self.stuck_retries = 0
                self.publish_state()
                self.publish_alert("Robot entered IDLE to conserve battery")
                if self.stuck_timer:
                    self.stuck_timer.cancel()
                    self.stuck_timer = None
            else:
                self.stuck_retries += 1
                self.get_logger().info(f"Attempting stuck recovery... (Retry {self.stuck_retries}/4)")
                self.state = "AUTONOMOUS_NAVIGATION"
                self.publish_state()
                self.publish_alert("Navigation path established")
                self.send_nav_goal(self.target_destination_name, self.target_coords)
        else:
            # If state changed manually (e.g. automatic or forward sent), stop timer
            if self.stuck_timer:
                self.stuck_timer.cancel()
                self.stuck_timer = None

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def control_loop(self):
        twist = Twist()
        
        if self.state == "MOVING_FORWARD":
            twist.linear.x = self.linear_speed
        elif self.state == "MOVING_BACK":
            twist.linear.x = -self.linear_speed
        elif self.state == "MOVING_LEFT":
            twist.angular.z = self.angular_speed
        elif self.state == "MOVING_RIGHT":
            twist.angular.z = -self.angular_speed
        else:
            pass

        if self.state in ["MOVING_FORWARD", "MOVING_BACK", "MOVING_LEFT", "MOVING_RIGHT"]:
            self.cmd_vel_pub.publish(twist)

    def scan_callback(self, msg):
        if self.state not in ["MOVING_FORWARD", "MOVING_BACK", "MOVING_LEFT", "MOVING_RIGHT"]:
            return
            
        # Determine cone based on state
        min_angle_deg, max_angle_deg = 0, 0
        if self.state == "MOVING_FORWARD":
            min_angle_deg, max_angle_deg = -30, 30
        elif self.state == "MOVING_BACK":
            min_angle_deg, max_angle_deg = 150, -150
        elif self.state == "MOVING_LEFT":
            min_angle_deg, max_angle_deg = 60, 120
        elif self.state == "MOVING_RIGHT":
            min_angle_deg, max_angle_deg = -120, -60

        safe = self.check_cone_safe(msg, min_angle_deg, max_angle_deg)
        
        if not safe:
            self.get_logger().warn(f"Obstacle detected! Stopping from state: {self.state}")
            self.state = "IDLE"
            self.stop_robot()
            self.publish_state()

    def check_cone_safe(self, msg, min_angle_deg, max_angle_deg):
        for i, range_val in enumerate(msg.ranges):
            # Ignore invalid readings
            if math.isinf(range_val) or math.isnan(range_val) or range_val < msg.range_min or range_val > msg.range_max:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)
            
            # Normalize angle to [-180, 180]
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360
                
            in_cone = False
            if min_angle_deg > max_angle_deg:
                # Rear cone wrap-around (e.g. 150 to -150 means 150 to 180 and -180 to -150)
                if angle_deg >= min_angle_deg or angle_deg <= max_angle_deg:
                    in_cone = True
            else:
                if angle_deg >= min_angle_deg and angle_deg <= max_angle_deg:
                    in_cone = True
                    
            if in_cone:
                if range_val < self.safety_distance:
                    return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()