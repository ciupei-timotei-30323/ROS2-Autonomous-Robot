'''
	This code interfaces with the STM32 via SPI to control the DDSM115 motors.
	It sends a custom 11-byte command and receives 11-byte feedback.
	Real RPM is derived by differentiating encoder ticks.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool
import time
import spidev
import numpy as np

class VelocityControl(Node):

	def __init__(self):
		super().__init__("velocity_control_node")
		self.get_logger().info('Start velocity_control_node')

		# ROS Parameters
		self.declare_parameter('encoder_cpr', 4096.0) # Pulses per revolution, adjust if RPM is inaccurate
		self.encoder_cpr = self.get_parameter('encoder_cpr').get_parameter_value().double_value

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

		# Encoder Tracking
		self.enc1_val_old = 0
		self.enc2_val_old = 0
		self.enc1_acc = 0
		self.enc2_acc = 0
		
		self.prev_enc1_total = 0
		self.prev_enc2_total = 0
		self.prev_time = time.time()

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
		current_time = time.time()
		if (current_time - self.last_rpm_recv_stamp) > 2.0 or self.brake_enable:
			target_left, target_right = 0, 0
		else:
			# Map the 4-wheel commands to Left and Right sides (average or use front wheels)
			target_left = self.rpm_cmd_list[0]
			target_right = self.rpm_cmd_list[1]

		# Clamp speeds to -100 to 100 as per MCU script
		left_speed = max(-100, min(100, target_left))
		right_speed = max(-100, min(100, target_right))

		# Construct the 11-byte array
		payload = [
			left_speed & 0xFF,  # byte 0: mSpdRef[0] (Left)
			0,                  # byte 1
			0,                  # byte 2
			right_speed & 0xFF, # byte 3: mSpdRef[3] (Right)
			0,                  # byte 4: relLedCtlBin
			0,                  # byte 5: ackVal
			0,                  # byte 6: 0
			0,                  # byte 7: rcVal
			8,                  # byte 8
			9,                  # byte 9
			10                  # byte 10
		]

		try:
			# xfer2 sends the payload and SIMULTANEOUSLY returns the received bytes
			response = self.spi.xfer2(payload)
			
			if len(response) == 11:
				# Parse ENC1 (bytes 0-1)
				enc1_raw = (response[0] << 8) | response[1]
				# Manage ENC1 overflow
				if enc1_raw < 1000 and self.enc1_val_old > 64000:
					self.enc1_acc += 65536
				elif enc1_raw > 64000 and self.enc1_val_old < 1000:
					self.enc1_acc -= 65536
				self.enc1_val_old = enc1_raw
				enc1_total = self.enc1_acc + enc1_raw

				# Parse ENC2 (bytes 2-3)
				enc2_raw = (response[2] << 8) | response[3]
				# Manage ENC2 overflow
				if enc2_raw < 1000 and self.enc2_val_old > 64000:
					self.enc2_acc += 65536
				elif enc2_raw > 64000 and self.enc2_val_old < 1000:
					self.enc2_acc -= 65536
				self.enc2_val_old = enc2_raw
				enc2_total = self.enc2_acc + enc2_raw

				# Calculate Delta Time in minutes
				dt_minutes = (current_time - self.prev_time) / 60.0
				if dt_minutes > 0:
					# RPM = (delta ticks / CPR) / (delta time in minutes)
					rpm_left = ((enc1_total - self.prev_enc1_total) / self.encoder_cpr) / dt_minutes
					rpm_right = ((enc2_total - self.prev_enc2_total) / self.encoder_cpr) / dt_minutes
				else:
					rpm_left = 0
					rpm_right = 0

				self.prev_enc1_total = enc1_total
				self.prev_enc2_total = enc2_total
				self.prev_time = current_time

				# Publish RPM Feedback mapping Left to indices 0,2 and Right to indices 1,3
				rpm_msg = Int16MultiArray()
				rpm_msg.data = [int(rpm_left), int(rpm_right), int(rpm_left), int(rpm_right)]
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