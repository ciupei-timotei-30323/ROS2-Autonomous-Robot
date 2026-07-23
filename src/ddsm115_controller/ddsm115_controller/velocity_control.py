'''
	This code interfaces with the STM32 via SPI to control the DDSM115 motors.
	It sends a custom 11-byte command and receives 11-byte feedback.
	Real RPM is derived by differentiating encoder ticks.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool, Int32, Float32
import time
import spidev
import numpy as np

class VelocityControl(Node):

	def __init__(self):
		super().__init__("velocity_control_node")
		self.get_logger().info('Start velocity_control_node')

		# ROS Parameters
		# TODO: Verify if the STM32/DDSM115 encoder setup is exactly 4096 pulses per revolution. 
		# If this is incorrect, all odometry distance calculations will be completely wrong.
		# On Google, it says DDSM115 encoder's CPR is 4096.
		self.declare_parameter('encoder_cpr', 4096.0) # Pulses per revolution, adjust if RPM is inaccurate
		self.encoder_cpr = self.get_parameter('encoder_cpr').get_parameter_value().double_value

		# CS Pin Configuration
		self.declare_parameter('cs_pin', 8) # Physical Pin 24 is BCM 8
		self.cs_pin = self.get_parameter('cs_pin').get_parameter_value().integer_value

		# Initialize GPIO for custom CS
		try:
			import RPi.GPIO as GPIO
			self.GPIO = GPIO
			self.GPIO.setmode(self.GPIO.BCM)
			self.GPIO.setwarnings(False)
			self.GPIO.setup(self.cs_pin, self.GPIO.OUT)
			self.GPIO.output(self.cs_pin, self.GPIO.HIGH)
			self.get_logger().info(f"Using custom CS pin (BCM {self.cs_pin}) via RPi.GPIO")
		except ImportError:
			self.GPIO = None
			self.get_logger().warn("RPi.GPIO not installed. Custom CS control disabled. Falling back to hardware CS.")

		# Initialize SPI
		try:
			self.spi = spidev.SpiDev()
			self.spi.open(0, 0) # Use CE0 as hardware fallback if no_cs fails
			self.spi.max_speed_hz = 100000 # Lower to 100kHz for better signal integrity over jumper wires
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

		self.pub_right_back_1 = self.create_publisher(Int32, "/sonar/right_back_1", qos_profile=qos)
		self.pub_right_back_2 = self.create_publisher(Int32, "/sonar/right_back_2", qos_profile=qos)
		self.pub_right_front = self.create_publisher(Int32, "/sonar/right_front", qos_profile=qos)
		self.pub_left_front = self.create_publisher(Int32, "/sonar/left_front", qos_profile=qos)
		self.pub_left_side = self.create_publisher(Int32, "/sonar/left_side", qos_profile=qos)
		self.pub_right_side = self.create_publisher(Int32, "/sonar/right_side", qos_profile=qos)

		self.yaw_pub = self.create_publisher(Float32, "/mcu/yaw", qos_profile=qos)

		self.timer =  self.create_timer(0.01, self.timer_callback)

	def rpm_cmd_callback(self, msg):
		for i, data in enumerate(msg.data):
			if i < 4:
				self.rpm_cmd_list[i] = data
		self.last_rpm_recv_stamp = time.time()
		self.get_logger().info(f"Received RPM Cmd from ROS: {self.rpm_cmd_list}")

	def brake_cmd_callback(self, msg):
		self.get_logger().info("Got brake {}".format(msg.data))
		self.brake_enable = msg.data

	def timer_callback(self):
		current_time = time.time()
		if (current_time - self.last_rpm_recv_stamp) > 2.0 or self.brake_enable:
			left_speed = 0
			right_speed = 0
		else:
			left_speed = self.rpm_cmd_list[0]
			right_speed = self.rpm_cmd_list[1]

		# Apply empirical 4-wheel mapping based on user tests:
		# In the last test, pressing forward made the robot go backward.
		# Inverting all 4 assignments from the last test gives us the perfect forward kinematics:
		target_br = right_speed
		target_bl = left_speed
		target_fl = left_speed
		target_fr = right_speed

		# Clamp speeds to the maximum allowed by the STM32 (-127 to 127)
		target_br = max(-127, min(127, int(target_br)))
		target_bl = max(-127, min(127, int(target_bl)))
		target_fl = max(-127, min(127, int(target_fl)))
		target_fr = max(-127, min(127, int(target_fr)))

		# Construct the 23-byte array for the STM32
		payload = [
			target_br & 0xFF,    # byte 0: Back Right
			target_bl & 0xFF,    # byte 1: Back Left
			target_fl & 0xFF,    # byte 2: Front Left
			target_fr & 0xFF,    # byte 3: Front Right
			0,                   # byte 4: relLedCtlBin
			1,                   # byte 5: ackVal - set to 1 to clear faults
			0,                   # byte 6: 0
			0,                   # byte 7: rcVal
			8,                   # byte 8
			9,                   # byte 9
			10                   # byte 10
		] + [0] * 12

		try:
			# Store a copy of the payload to log it correctly since xfer2 overwrites the list
			payload_tx = payload.copy()
			
			if self.GPIO is not None:
				self.GPIO.output(self.cs_pin, self.GPIO.LOW)
				time.sleep(0.0001) # 100us delay to let STM32 SPI slave setup
				
			response = self.spi.xfer2(payload)
			
			if self.GPIO is not None:
				self.GPIO.output(self.cs_pin, self.GPIO.HIGH)
				
			if len(response) == 23:
				# Log the raw SPI response occasionally or always if debugging
				self.get_logger().info(f"SPI Tx: {payload_tx} | Rx: {response}")
				
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

				# Parse and publish sonars
				msg = Int32()
				
				msg.data = (response[11] << 8) | response[12]
				self.pub_right_back_1.publish(msg)

				msg.data = (response[13] << 8) | response[14]
				self.pub_right_back_2.publish(msg)

				msg.data = (response[15] << 8) | response[16]
				self.pub_right_front.publish(msg)

				msg.data = (response[17] << 8) | response[18]
				self.pub_left_front.publish(msg)

				msg.data = (response[19] << 8) | response[20]
				self.pub_left_side.publish(msg)

				msg.data = (response[21] << 8) | response[22]
				self.pub_right_side.publish(msg)

				# Parse and publish MCU Yaw
				yaw_raw = (response[6] << 8) | response[7]
				if yaw_raw > 32767:
					yaw_raw -= 65536
				yaw_msg = Float32()
				yaw_msg.data = float(yaw_raw)
				self.yaw_pub.publish(yaw_msg)

		except Exception as e:
			self.get_logger().error(f"SPI Error: {e}")

	def __del__(self):
		if hasattr(self, 'spi'):
			self.spi.close()
		if hasattr(self, 'GPIO') and self.GPIO is not None and hasattr(self, 'cs_pin'):
			self.GPIO.cleanup(self.cs_pin)

def main(args=None):
	rclpy.init(args=args)
	node = VelocityControl()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()