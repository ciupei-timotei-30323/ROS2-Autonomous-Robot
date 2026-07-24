import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class YawToPoseNode(Node):
    def __init__(self):
        super().__init__('yaw_to_pose_node')
        
        # MCU yaw scale (e.g. if the MCU sends degrees, set this to pi/180)
        # Assuming radians by default based on typical ROS usage unless scaled elsewhere
        self.declare_parameter("yaw_mcu_scale", 1.0)
        self.yaw_mcu_scale = self.get_parameter('yaw_mcu_scale').get_parameter_value().double_value
        
        # Subscribe to MCU yaw
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, 
            depth=1
        )
        self.yaw_sub = self.create_subscription(Float32, '/mcu/yaw', self.yaw_callback, qos_profile=qos)
        
        # Publish PoseWithCovarianceStamped
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/yaw_pose', 10)
        self.get_logger().info("YawToPoseNode started. Translating /mcu/yaw to /yaw_pose for EKF")

    def yaw_callback(self, msg):
        yaw = msg.data * self.yaw_mcu_scale
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "odom"
        
        # We only care about orientation (yaw)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Higher covariance for yaw to prevent snapping on bumps, rely on wheels for short-term rotation
        pose_msg.pose.covariance[35] = 0.1  # yaw covariance
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YawToPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
