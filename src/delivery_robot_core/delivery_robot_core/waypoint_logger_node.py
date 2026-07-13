#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import threading
import json
import math
import sys
import time
import os
from ament_index_python.packages import get_package_share_directory

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        default_locations_file = os.path.join(
            get_package_share_directory('delivery_robot_core'),
            'config',
            'Locations.json'
        )
        self.declare_parameter('locations_file_path', default_locations_file)
        self.filepath = self.get_parameter('locations_file_path').value

    def get_robot_pose(self):
        try:
            # wait for transform to be available
            now = rclpy.time.Time()
            if not self.tf_buffer.can_transform('map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=2.0)):
                self.get_logger().error("Timeout waiting for transform from 'map' to 'base_link'")
                return None, None, None
                
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, theta
        except Exception as e:
            self.get_logger().error(f"Could not get transform: {e}")
            return None, None, None

    def save_location(self, name):
        x, y, theta = self.get_robot_pose()
        if x is None:
            print("Failed to get pose. Is SLAM running and map published?")
            return
            
        try:
            with open(self.filepath, 'r') as f:
                data = json.load(f)
        except:
            data = {}
            
        data[name] = {"x": round(x, 3), "y": round(y, 3), "theta": round(theta, 3)}
        
        try:
            with open(self.filepath, 'w') as f:
                json.dump(data, f, indent=4)
            print(f"\n--> Saved '{name}' at x={round(x,3)}, y={round(y,3)}, theta={round(theta,3)} to Locations.json!\n")
        except Exception as e:
            print(f"Failed to save to {self.filepath}: {e}")

def input_loop(node):
    print("========================================")
    print("Waypoint Logger Initialized.")
    print("Drive the robot to the desired location.")
    print("Type the name of the location and press Enter to save it.")
    print("Type 'q' or 'quit' to exit.")
    print("========================================\n")
    
    # Wait a bit for tf buffer to fill
    time.sleep(1.0)
    
    while rclpy.ok():
        try:
            user_input = input("Enter location name to save (or 'q' to quit): ").strip()
            if not user_input:
                continue
            if user_input.lower() in ['q', 'quit']:
                print("Exiting...")
                rclpy.shutdown()
                break
            
            node.save_location(user_input)
        except EOFError:
            break
        except Exception as e:
            print(f"Error: {e}")
            break

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    
    # Run the input loop in a separate thread so it doesn't block rclpy.spin
    input_thread = threading.Thread(target=input_loop, args=(node,))
    input_thread.daemon = True
    input_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        input_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()
