#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

class MissionCoordinatorNode(Node):
    def __init__(self):
        super().__init__('mission_coordinator_node')

        # --- FSM States ---
        self.state = "IDLE"  # States: IDLE, NAVIGATING, WAITING_FOR_AUTH

        # --- Hardcoded Office Locations ---
        # TODO : load from yaml, after slam generates the map and we have the coordinates
        self.locations = {
            "Desk 1": {"x": 2.5, "y": 1.0, "theta": 0.0},
            "Desk 2": {"x": -3.0, "y": 4.5, "theta": 1.57},
            "Base": {"x": 0.0, "y": 0.0, "theta": 0.0}
        }

        # --- Publishers & Subscribers ---
        # 1. Listen for voice destination from Jetson (via websocket)
        self.dest_sub = self.create_subscription(
            String,
            '/system/voice_destination',
            self.voice_dest_callback,
            10
        )

        # 2. Listen for face auth status from the external Pi (via websocket)
        self.auth_sub = self.create_subscription(
            Bool,
            '/system/face_auth_status',
            self.face_auth_callback,
            10
        )

        # 3. Publish robot state back to external systems (via websocket)
        self.state_pub = self.create_publisher(String, '/system/robot_state', 10)

        # --- Nav2 Action Client ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info("Mission Coordinator initialized and waiting in IDLE state.")
        self.publish_state()

    def publish_state(self):
        """Broadcasts current FSM state to the external systems."""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
        self.get_logger().info(f"State changed to: {self.state}")

    def voice_dest_callback(self, msg):
        """Triggered when the Jetson sends a location string."""
        destination_name = msg.data

        if self.state != "IDLE":
            self.get_logger().warn("Ignored voice command; robot is not IDLE.")
            return

        if destination_name not in self.locations:
            self.get_logger().error(f"Unknown destination: {destination_name}")
            return

        self.get_logger().info(f"Received dispatch to: {destination_name}")
        self.send_nav_goal(destination_name)

    def send_nav_goal(self, location_name):
        """Constructs the Nav2 goal and sends it asynchronously."""
        # Wait for the action server to be available
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Populate coordinates
        coords = self.locations[location_name]
        goal_msg.pose.pose.position.x = coords["x"]
        goal_msg.pose.pose.position.y = coords["y"]
        goal_msg.pose.pose.position.z = 0.0
        
        # Simplistic Euler to Quaternion for the Z-axis rotation (theta)
        import math
        goal_msg.pose.pose.orientation.z = math.sin(coords["theta"] / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(coords["theta"] / 2.0)

        self.get_logger().info(f"Sending goal to Nav2: {location_name}")
        
        # Send goal and attach callbacks for when it's accepted and finished
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.state = "NAVIGATING"
        self.publish_state()

    def goal_response_callback(self, future):
        """Triggered when Nav2 accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the navigation goal.")
            self.state = "IDLE"
            self.publish_state()
            return

        self.get_logger().info("Nav2 accepted goal. Driving...")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Triggered when the robot physically arrives at the destination."""
        result = future.result().result
        status = future.result().status

        if status == 4: # SUCCEEDED
            self.get_logger().info("Arrived at destination. Awaiting Face Auth.")
            self.state = "WAITING_FOR_AUTH"
        else:
            self.get_logger().error(f"Navigation failed with status code: {status}")
            self.state = "IDLE"
            
        self.publish_state()

    def face_auth_callback(self, msg):
        """Triggered when the external Pi publishes a boolean auth result."""
        if self.state != "WAITING_FOR_AUTH":
            return

        auth_success = msg.data
        if auth_success:
            self.get_logger().info("Face Auth Successful! Delivery complete. Returning to base.")
            # Trigger the return trip
            self.send_nav_goal("Base")
        else:
            self.get_logger().warn("Face Auth Failed. Still waiting...")

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