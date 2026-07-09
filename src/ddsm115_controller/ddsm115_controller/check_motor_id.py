'''
	This code is to check online motor ID without doing anything else.

	Developed by Rasheed Kittinanthapanya

	https://github.com/rasheeddo/
	https://www.youtube.com/@stepbystep-robotics
'''

import rclpy
from rclpy.node import Node
from ddsm115_controller.ddsm115 import *

class  CheckMotorId(Node):

	def __init__(self):
		super().__init__("check_motor_id_node")
		self.get_logger().info('Start check_motor_id_node')

		self.declare_parameter('max_check', 10)
		self.declare_parameter('usb_dev', "/dev/ttyUSB0")

		self.max_check = self.get_parameter('max_check').get_parameter_value().integer_value
		self.usb_dev = self.get_parameter('usb_dev').get_parameter_value().string_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("max_check: {}".format(self.max_check))
		self.get_logger().info("usb_dev: {}".format(self.usb_dev))

		d = MotorControl(self.usb_dev)
		online_id = []
		for i in range(self.max_check):
			data_fb = d.get_motor_feedback(i+1)
			if data_fb['id'] != None:
				online_id.append(data_fb['id'])

		self.get_logger().info("Online ID is {}".format(online_id))
		quit()

def main(args=None):
	rclpy.init(args=args)
	node = CheckMotorId()
	rclpy.spin(node)
	node.destroy()
	rclpy.shutdown()


if __name__ == '__main__':
	main()