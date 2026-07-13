#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket
import threading

class TcpBridgeNode(Node):
    def __init__(self):
        super().__init__('tcp_bridge_node')
        
        # Publishers matching your Mission Coordinator
        self.voice_pub = self.create_publisher(String, '/system/voice_destination', 10)
        self.face_pub = self.create_publisher(Bool, '/system/face_auth_status', 10)

        # Subscriber matching Mission Coordinator state
        self.state_sub = self.create_subscription(String, '/system/robot_state', self.state_callback, 10)
        
        # Subscriber for custom text alerts to send directly over TCP
        self.alert_sub = self.create_subscription(String, '/system/tcp_alert', self.alert_callback, 10)
        
        self.active_conn = None

        # Port for the TCP Server
        self.port = 5000 
        
        # Start the TCP server in a daemon thread, deferred to avoid rclpy logger race condition
        self.server_thread = None
        self.start_timer = self.create_timer(0.5, self.start_server_thread)

    def start_server_thread(self):
        if self.start_timer:
            self.start_timer.cancel()
        self.server_thread = threading.Thread(target=self.run_tcp_server, daemon=True)
        self.server_thread.start()

    def state_callback(self, msg):
        """Forwards the robot state to the connected TCP client."""
        if self.active_conn:
            try:
                # Add a newline so the receiver can easily parse the stream
                payload = f"STATE:{msg.data}\n".encode('utf-8')
                self.active_conn.sendall(payload)
                self.get_logger().info(f"Sent State over TCP: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"Failed to send state over TCP: {e}")

    def alert_callback(self, msg):
        """Forwards raw text alerts to the connected TCP client."""
        if self.active_conn:
            try:
                # Add a newline so the receiver can easily parse the stream
                payload = f"{msg.data}\n".encode('utf-8')
                self.active_conn.sendall(payload)
                self.get_logger().info(f"Sent Alert over TCP: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"Failed to send alert over TCP: {e}")

    def run_tcp_server(self):
        """Runs a raw TCP server listening for incoming connections."""
        host = '0.0.0.0' # Listen on all available network interfaces
        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, self.port))
            s.listen()
            self.get_logger().info(f"TCP Bridge listening on port {self.port} (Raw TCP)")
            
            while True:
                conn, addr = s.accept()
                self.get_logger().info(f"Connection established from {addr}")
                self.active_conn = conn
                with conn:
                    try:
                        conn.sendall(b"Connection established to ROS2\n")
                        while True:
                            # Receive up to 1024 bytes and decode to string
                            data = conn.recv(1024).decode('utf-8').strip()
                            if not data:
                                break  # Client disconnected
                            
                            # Handle potentially multiple messages split by newline
                            for line in data.split('\n'):
                                line = line.strip()
                                if line:
                                    self.process_data(line)
                    except Exception as e:
                        self.get_logger().warn(f"Connection error: {e}")
                self.active_conn = None
                self.get_logger().info(f"Connection closed from {addr}")

    def process_data(self, data):
        """
        Parses our custom minimalist protocol.
        Format expected: "VOICE: command" or "FACE:True" or "GO Desk 1"
        """
        if data.upper() == "STOP":
            msg = String()
            msg.data = "STOP"
            self.voice_pub.publish(msg)
            self.get_logger().info("Received STOP Command")
            return

        if data.upper().startswith("GO "):
            destination = data[3:].strip()
            msg = String()
            msg.data = f"GO {destination}"
            self.voice_pub.publish(msg)
            self.get_logger().info(f"Received GO Command: {destination}")
            return

        try:
            prefix, payload = data.split(':', 1)
            prefix = prefix.strip().upper()
            payload = payload.strip()
            
            if prefix == "VOICE":
                msg = String()
                msg.data = payload
                self.voice_pub.publish(msg)
                self.get_logger().info(f"Received Voice Command: {payload}")
                
            elif prefix == "FACE":
                msg = Bool()
                msg.data = (payload.lower() == "true")
                self.face_pub.publish(msg)
                self.get_logger().info(f"Received Face Auth: {msg.data}")
                
        except ValueError:
            self.get_logger().warn(f"Malformed TCP payload received: {data}")

def main(args=None):
    rclpy.init(args=args)
    node = TcpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()