'''
	This code interfaces with the STM32 via SPI to control the DDSM115 motors.
	The feedback of RPM is received simultaneously via full-duplex SPI.
	The command of RPM and brake are subscribed and wait for user to publish.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool
import time
import spidev
import struct

class VelocityControl(Node):

	def __init__(self):
		super().__init__("velocity_control_node")
		self.get_logger().info('Start velocity_control_node')

		# Initialize SPI
		try:
			self.spi = spidev.SpiDev()
			self.spi.open(0, 0)
			self.spi.max_speed_hz = 1000000
			self.spi.mode = 0
			self.get_logger().info("SPI initialized on /dev/spidev0.0")
		except Exception as e:
			self.get_logger().error(f"Failed to initialize SPI: {e}")
			raise e

		### Variables ###
		self.rpm_cmd_list = [0, 0, 0, 0]
		self.last_rpm_recv_stamp = time.time()
		self.brake_enable = False

		### Pub/Sub ###
		qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
											history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
											depth=1)
		self.rpm_cmd_sub = self.create_subscription(Int16MultiArray, "/ddsm115/rpm_cmd", self.rpm_cmd_callback, qos_profile=qos)
		self.brake_cmd_sub = self.create_subscription(Bool, "/ddsm115/brake", self.brake_cmd_callback, qos_profile=qos)
		
		self.rpm_fb_pub = self.create_publisher(Int16MultiArray, "/ddsm115/rpm_fb", qos_profile=qos)

		self.get_logger().info('----------------------------------- Publishers --------------------------------------')
		self.get_logger().info('Publish motors rpm feedback to      /ddsm115/rpm_fb     [std_msgs/msg/Int16MultiArray]')
		self.get_logger().info('---------------------------------- Subscribers --------------------------------------')
		self.get_logger().info('Subscribe motors rpm cmd at         /ddsm115/rpm_cmd    [std_msgs/msg/Int16MultiArray]')
		self.get_logger().info('Subscribe motors brake cmd at       /ddsm115/brake      [std_msgs/msg/Bool]')

		self.timer =  self.create_timer(0.01, self.timer_callback)

	def rpm_cmd_callback(self, msg):
		for i, data in enumerate(msg.data):
			if i < 4:
				self.rpm_cmd_list[i] = data
		self.last_rpm_recv_stamp = time.time()

	def brake_cmd_callback(self, msg):
		self.get_logger().info("Got brake {}".format(msg.data))
		self.brake_enable = msg.data

	def timer_callback(self):
		if (time.time() - self.last_rpm_recv_stamp) > 2.0 or self.brake_enable:
			v1, v2, v3, v4 = 0, 0, 0, 0
		else:
			v1, v2, v3, v4 = self.rpm_cmd_list[0], self.rpm_cmd_list[1], self.rpm_cmd_list[2], self.rpm_cmd_list[3]

		# Pack the velocities into binary
		try:
			data = struct.pack('<hhhh', v1, v2, v3, v4)
			payload = list(data)
			
			# xfer2 sends the payload and SIMULTANEOUSLY returns the received bytes
			response = self.spi.xfer2(payload)
			
			# Unpack the received 8 bytes back into 4 integers
			actual_speeds = struct.unpack('<hhhh', bytes(response))

			rpm_msg = Int16MultiArray()
			rpm_msg.data = list(actual_speeds)
			self.rpm_fb_pub.publish(rpm_msg)

		except Exception as e:
			self.get_logger().error(f"SPI Error: {e}")

	def __del__(self):
		if hasattr(self, 'spi'):
			self.spi.close()

def main(args=None):
	rclpy.init(args=args)
	node = VelocityControl()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy()
		rclpy.shutdown()

if __name__ == '__main__':
	main()