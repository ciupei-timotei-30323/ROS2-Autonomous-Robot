import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int8MultiArray, Float32MultiArray, Bool, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import numpy as np
import time
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class FourWheelsRobot(Node):

	def __init__(self):
		super().__init__("four_wheels_robot_node")
		self.get_logger().info('Start four_wheels_robot_node')

		### ROS Parameters ###
		self.declare_parameter("wheel_base", 0.225) # TODO: measure it
		self.declare_parameter("R_wheel", 0.05035) 
		self.declare_parameter("pub_tf", False)
		self.declare_parameter("yaw_odom_scale", 1.0) # Scale factor for angular odometry to compensate for skid-steer slip
		
		self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
		self.R_wheel = self.get_parameter('R_wheel').get_parameter_value().double_value
		self.pub_tf = self.get_parameter('pub_tf').get_parameter_value().bool_value
		self.yaw_odom_scale = self.get_parameter('yaw_odom_scale').get_parameter_value().double_value

		self.declare_parameter("yaw_mcu_scale", 1.0) # Set this to e.g. 0.1 or math.pi/180.0 if the MCU sends scaled degrees
		self.yaw_mcu_scale = self.get_parameter('yaw_mcu_scale').get_parameter_value().double_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("wheel_base: {}".format(self.wheel_base))
		self.get_logger().info("R_wheel: {}".format(self.R_wheel))
		self.get_logger().info("pub_tf: {}".format(self.pub_tf))
		self.get_logger().info("yaw_odom_scale: {}".format(self.yaw_odom_scale))
		self.get_logger().info("yaw_mcu_scale: {}".format(self.yaw_mcu_scale))

		### Variables ###
		self.rpm_fb_left = 0.0
		self.rpm_fb_right = 0.0
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.yaw_mcu = 0.0
		self.period = 0.01
		self.prev_y = 0.0

		self.thr_stick = 0.0
		self.str_stick = 0.0
		self.cart_mode = 2
		self.prev_cart_mode = self.cart_mode

		self.max_rpm = 150

		### TF ###
		self.br = TransformBroadcaster(self)

		### Pub/Sub ###
		qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
											history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
											depth=1)
		self.rpm_cmd_pub = self.create_publisher(Int16MultiArray, "/ddsm115/rpm_cmd", qos_profile=qos)
		self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

		self.rpm_fb_sub = self.create_subscription(Int16MultiArray, "/ddsm115/rpm_fb", self.rpm_fb_callback, qos_profile=qos)
		self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
		self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
		self.yaw_sub = self.create_subscription(Float32, "/mcu/yaw", self.yaw_callback, qos_profile=qos)

		self.timer = self.create_timer(self.period, self.timer_callback)

	def _print(self, msg):
		self.get_logger().info("{}".format(msg))

	#####################
	### ROS Callbacks ###
	#####################
	def rpm_fb_callback(self, msg):
		# Average the two left wheels (index 0: FL, index 2: RL)
		# Average the two right wheels (index 1: FR, index 3: RR), inverted as per original logic
		if len(msg.data) >= 4:
			self.rpm_fb_left = (msg.data[0] + msg.data[2]) / 2.0
			self.rpm_fb_right = (-msg.data[1] - msg.data[3]) / 2.0

	def cmd_vel_callback(self, msg):
		vx = msg.linear.x
		wz = msg.angular.z

		# Use the same simple kinematics as the colleagues' code
		vl = vx - wz * self.wheel_base / 2.0
		vr = vx + wz * self.wheel_base / 2.0

		# Convert to percentages based on max speed (e.g., 0.80 m/s)
		max_wheel_speed = 0.80
		
		# Prevent divide by zero if max_wheel_speed is misconfigured
		if max_wheel_speed > 0:
			left_pct = int(max(-100, min(100, vl / max_wheel_speed * 100.0)))
			right_pct = int(max(-100, min(100, vr / max_wheel_speed * 100.0)))
		else:
			left_pct = 0
			right_pct = 0

		left_rpm = left_pct
		right_rpm = -right_pct

		rpm_cmd_msg = Int16MultiArray()
		# Format: [front_left, front_right, rear_left, rear_right]
		rpm_cmd_msg.data = [left_rpm, right_rpm, left_rpm, right_rpm]

		self.rpm_cmd_pub.publish(rpm_cmd_msg)
		self._print("Cmd_vel vx: {:.2f} wz: {:.2f} vl: {:.2f} vr: {:.2f} rpmL: {:d} rpmR: {:d}".format(\
			vx, wz, vl, vr, left_rpm, right_rpm))
		
	def joy_callback(self, msg):

		self.thr_stick = msg.axes[1]*100.0
		self.str_stick = -msg.axes[3]*100.0

		## A button AUTO
		if (msg.buttons[0] == 1):
			self.prev_cart_mode = self.cart_mode
			self.cart_mode = 2
			self._print("cart mode is 2")

		## X button MANUAL
		elif (msg.buttons[2] == 1):
			self.prev_cart_mode = self.cart_mode
			self.cart_mode = 1
			self._print("cart mode is 1")

	def yaw_callback(self, msg):
		self.yaw_mcu = msg.data * self.yaw_mcu_scale

	######################
	### Math functions ###
	######################
	def map_with_limit(self, val, in_min, in_max, out_min, out_max):

		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		if out_min > out_max:
			if out > out_min:
				out = out_min
			elif out < out_max:
				out = out_max
			else:
				pass
		elif out_max > out_min:
			if out > out_max:
				out = out_max
			elif out < out_min:
				out = out_min
			else:
				pass
		else:
			pass

		return out

	def euler_to_quaternion(self, roll, pitch, yaw):
		qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
		qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
		qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		return [qx, qy, qz, qw]

	def linear_to_rpm(self, v):
		return (60.0/(2.0*np.pi))*(v/self.R_wheel)

	def rpm_to_linear(self, rpm):
		return rpm * ((2.0*np.pi)/60.0) * self.R_wheel
	
	def xy_mixing(self, x, y):
		## x, y must be in the range of -100 to 100

		left = y+x
		right = y-x

		diff = abs(x) - abs(y)

		if (left < 0.0):
			left = left - abs(diff)
		else:
			left = left + abs(diff)

		if (right < 0.0):
			right = right - abs(diff)
		else:
			right = right + abs(diff)

		if (self.prev_y < 0.0):
			swap = left
			left = right
			right = swap
		
		self.prev_y = y

		## left and right are in -200 to 200 ranges

		return left, right


	############
	### Loop ###
	############	
	def timer_callback(self):

		if self.cart_mode == 1:
			if (-5.0 <= self.thr_stick <= 5.0) and (-5.0 <= self.str_stick <= 5.0):
				left_rpm = 0
				right_rpm = 0
			else:
				left_200_per, right_200_per = self.xy_mixing(self.str_stick, self.thr_stick)
				left_rpm = int(self.map_with_limit(left_200_per, -200.0, 200.0, -self.max_rpm, self.max_rpm))
				right_rpm = int(self.map_with_limit(right_200_per, -200.0, 200.0, self.max_rpm, -self.max_rpm))

			rpm_cmd_msg = Int16MultiArray()
			# Format: [front_left, front_right, rear_left, rear_right]
			rpm_cmd_msg.data = [left_rpm, right_rpm, left_rpm, right_rpm]
			self.rpm_cmd_pub.publish(rpm_cmd_msg)

			self._print("Joystick str_stick: {:.2f} thr_stick: {:.2f} rpmL: {:d} rpmR: {:d}".format(\
				self.str_stick, self.thr_stick, left_rpm, right_rpm))



		### Calculate ODOM ###
		vl = round(self.rpm_to_linear(self.rpm_fb_left), 3)
		vr = round(self.rpm_to_linear(self.rpm_fb_right), 3)

		# If wheels are moving very slowly or stopped, force V to 0
		if abs(vl) < 0.01 and abs(vr) < 0.01:
			V = 0.0
		else:
			V = (vl + vr)/2.0

		# Read the new yaw from the MCU
		new_theta = self.yaw_mcu
		
		# Calculate angular velocity (Wz) based on the change in MCU Yaw
		diff = new_theta - self.theta
		diff = math.atan2(math.sin(diff), math.cos(diff)) # normalize to -pi to pi
		
		Wz = diff / self.period
		
		# Position update (X, Y) using the new average heading
		avg_theta = self.theta + diff / 2.0
		self.x = self.x + V * np.cos(avg_theta) * self.period
		self.y = self.y + V * np.sin(avg_theta) * self.period
		
		# Update orientation
		self.theta = new_theta

		q = self.euler_to_quaternion(0,0, self.theta)
		odom_msg = Odometry()
		odom_msg.header.stamp = self.get_clock().now().to_msg()
		odom_msg.header.frame_id = "odom"
		odom_msg.child_frame_id = "base_link"	
		odom_msg.pose.pose.position.x = self.x
		odom_msg.pose.pose.position.y = self.y
		odom_msg.pose.pose.position.z = 0.0
		odom_msg.pose.pose.orientation.x = q[0]
		odom_msg.pose.pose.orientation.y = q[1]
		odom_msg.pose.pose.orientation.z = q[2]
		odom_msg.pose.pose.orientation.w = q[3]
		odom_msg.pose.covariance[0] = 0.0001
		odom_msg.pose.covariance[7] = 0.0001
		odom_msg.pose.covariance[14] = 0.000001	#1e12
		odom_msg.pose.covariance[21] = 0.000001	#1e12
		odom_msg.pose.covariance[28] = 0.000001	#1e12
		odom_msg.pose.covariance[35] = 0.0001
		odom_msg.twist.twist.linear.x = V 
		odom_msg.twist.twist.linear.y = 0.0 
		odom_msg.twist.twist.angular.z = Wz 
		self.odom_pub.publish(odom_msg)

		if self.pub_tf:
			# construct tf
			t = TransformStamped()
			t.header.frame_id = "odom" 
			t.header.stamp = self.get_clock().now().to_msg()
			t.child_frame_id = "base_link"	
			t.transform.translation.x = self.x 
			t.transform.translation.y = self.y
			t.transform.translation.z = 0.0

			t.transform.rotation.x = odom_msg.pose.pose.orientation.x
			t.transform.rotation.y = odom_msg.pose.pose.orientation.y
			t.transform.rotation.z = odom_msg.pose.pose.orientation.z
			t.transform.rotation.w = odom_msg.pose.pose.orientation.w
			self.br.sendTransform(t)

def main(args=None):
	rclpy.init(args=args)
	node = FourWheelsRobot()
	rclpy.spin(node)
	node.destroy()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
