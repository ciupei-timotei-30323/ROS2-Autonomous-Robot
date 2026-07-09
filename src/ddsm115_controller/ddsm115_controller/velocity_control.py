'''
	This code is to interface with DDSM115 python driver.
	The feedback of RPM, temp, current, temp are published continuously as std_msgs.
	The command of RPM and brake are subscribed and wait for user to publish.

	Developed by Rasheed Kittinanthapanya

	https://github.com/rasheeddo/
	https://www.youtube.com/@stepbystep-robotics
'''

import rclpy
from rclpy.node import Node
from ddsm115_controller.ddsm115 import *
from std_msgs.msg import Int16MultiArray, Int8MultiArray, Float32MultiArray, Bool
import time

class VelocityControl(Node):

	def __init__(self):
		super().__init__("velocity_control_node")
		self.get_logger().info('Start velocity_control_node')

		self.declare_parameter('max_check', 10)
		self.declare_parameter('usb_dev', "/dev/ttyUSB0")

		self.max_check = self.get_parameter('max_check').get_parameter_value().integer_value
		self.usb_dev = self.get_parameter('usb_dev').get_parameter_value().string_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("max_check: {}".format(self.max_check))
		self.get_logger().info("usb_dev: {}".format(self.usb_dev))

		### Check available motors ###
		self.driver = MotorControl(self.usb_dev)
		self.online_id = []
		for i in range(self.max_check):
			data_fb = self.driver.get_motor_feedback(i+1)
			if data_fb['id'] != None:
				self.online_id.append(data_fb['id'])

		self.get_logger().info("Online ID is {}".format(self.online_id))
		self.total_motor = len(self.online_id)
		self.last_motor_id = max(self.online_id)

		if (self.total_motor == 0):
			self.get_logger().info("No motor detected...")
			quit()

		for motor_id in self.online_id:
			ret = self.driver.set_drive_mode(_id=motor_id, _mode=2)
			self.get_logger().info("{}".format(ret))

		### Variables ###
		self.rpm_cmd_list = [None]*10
		self.brake_cmd_list = [None]*10
		self.last_rpm_recv_stamp = time.time()
		self.brake_enable = False
		self.last_slow_pub_stamp = time.time()

		self.rpm_fb_list = [0]*self.last_motor_id
		self.temp_fb_list = [0]*self.last_motor_id
		self.cur_fb_list = [0.0]*self.last_motor_id
		self.error_list = [0]*self.last_motor_id
		
		### Pub/Sub ###
		qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
											history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
											depth=1)
		self.rpm_cmd_sub = self.create_subscription(Int16MultiArray, "/ddsm115/rpm_cmd", self.rpm_cmd_callback, qos_profile=qos)
		self.brake_cmd_sub = self.create_subscription(Bool, "/ddsm115/brake", self.brake_cmd_callback, qos_profile=qos)
		
		self.rpm_fb_pub = self.create_publisher(Int16MultiArray, "/ddsm115/rpm_fb", qos_profile=qos)
		self.cur_fb_pub = self.create_publisher(Float32MultiArray, "/ddsm115/cur_fb", qos_profile=qos)
		self.temp_fb_pub = self.create_publisher(Int8MultiArray, "/ddsm115/temp_fb", qos_profile=qos)
		self.error_pub = self.create_publisher(Int8MultiArray, "/ddsm115/error", qos_profile=qos)
		self.motor_online_pub = self.create_publisher(Int8MultiArray, "/ddsm115/online_id", qos_profile=qos)

		self.get_logger().info('----------------------------------- Publishers --------------------------------------')
		self.get_logger().info('Publish motors rpm feedback to      /ddsm115/rpm_fb     [std_msgs/msg/Int16MultiArray]')
		self.get_logger().info('Publish motors current feedback to  /ddsm115/cur_fb     [std_msgs/msg/Float32MultiArray]')
		self.get_logger().info('Publish motors temp feedback to     /ddsm115/temp_fb    [std_msgs/msg/Int8MultiArray]')
		self.get_logger().info('Publish motors error feedback to    /ddsm115/error      [std_msgs/msg/Int8MultiArray]')
		self.get_logger().info('Publish motors online id to         /ddsm115/online_id  [std_msgs/msg/Int8MultiArray]')
		self.get_logger().info('---------------------------------- Subscribers --------------------------------------')
		self.get_logger().info('Subscribe motors rpm cmd at         /ddsm115/rpm_cmd    [std_msgs/msg/Int16MultiArray]')
		self.get_logger().info('Subscribe motors brake cmd at       /ddsm115/brake      [std_msgs/msg/Bool]')

		self.timer =  self.create_timer(0.01, self.timer_callback)

	#####################
	### ROS callbacks ###
	#####################
	def rpm_cmd_callback(self, msg):
		for i, data in enumerate(msg.data):
			self.rpm_cmd_list[i] = data

		self.last_rpm_recv_stamp = time.time()

	def brake_cmd_callback(self, msg):
		self.get_logger().info("Got brake {}".format(msg.data))
		if msg.data:
			self.brake_enable = True
		else:
			self.brake_enable = False

	##############
	### Helper ###
	##############
	def set_rpm(self):
		counter = 0
		self.brake_enable = False
		for count, rpm_cmd in enumerate(self.rpm_cmd_list):
			_id = count+1
			if (rpm_cmd is not None) and (_id in self.online_id):
				
				self.driver.send_rpm(_id, rpm_cmd)
				counter += 1

				## once set all the total motor, doesn't need to finish for loop
				if counter == self.total_motor:
					break

	def set_zero_rpm(self):
		for motor_id in self.online_id:
			self.rpm_cmd_list[motor_id-1] = 0

		self.set_rpm()

	def brake_motors(self):
		for motor_id in self.online_id:
			self.driver.set_brake(_id=motor_id)

	############
	### Loop ###
	############
	def timer_callback(self):

		if (time.time() - self.last_rpm_recv_stamp) > 2.0:
			if not self.brake_enable:
				self.set_zero_rpm()
			else:
				self.brake_motors()
		else:
			self.set_rpm()


		## Report feedback and status ##
		## list size will be equal to number of last id
		## e.g. last_motor_id = 3 => rpm_fb_list = [0,0,0], even 1 or 2 is not online
		
		for motor_id in self.online_id:
			data_fb = self.driver.get_motor_feedback(_id=motor_id)
			if (data_fb['id'] != None):
				self.rpm_fb_list[motor_id-1] = data_fb['fb_rpm']
				self.temp_fb_list[motor_id-1] = data_fb['wind_temp']
				self.cur_fb_list[motor_id-1] =  data_fb['fb_cur']
				self.error_list[motor_id-1] = data_fb['error']

		
		rpm_msg = Int16MultiArray()
		rpm_msg.data = self.rpm_fb_list
		self.rpm_fb_pub.publish(rpm_msg)

		cur_msg = Float32MultiArray()
		cur_msg.data = self.cur_fb_list
		self.cur_fb_pub.publish(cur_msg)

		if (time.time() - self.last_slow_pub_stamp) > 0.1:
			## some data doesn't need to publish too frequent ##
			temp_msg = Int8MultiArray()
			temp_msg.data = self.temp_fb_list
			self.temp_fb_pub.publish(temp_msg)

			error_msg = Int8MultiArray()
			error_msg.data = self.error_list
			self.error_pub.publish(error_msg)

			online_id_msg = Int8MultiArray()
			online_id_msg.data = self.online_id
			self.motor_online_pub.publish(online_id_msg)

			self.last_slow_pub_stamp = time.time()
	

		

def main(args=None):
	rclpy.init(args=args)
	node = VelocityControl()
	rclpy.spin(node)
	node.destroy()
	rclpy.shutdown()


if __name__ == '__main__':
	main()