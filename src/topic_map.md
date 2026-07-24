# ROS 2 Topic & Node Map

This document maps out the current architecture of the robot's nodes and the topics they publish and subscribe to, reflecting the recent EKF and Kinematics integration.

## 1. Hardware & Core Nodes

### `velocity_control_node` (Package: `ddsm115_controller`)
*Interfaces with the STM32 via SPI to control the DDSM115 motors and read sensors.*
- **Subscribes To:**
  - `/ddsm115/rpm_cmd` (`std_msgs/Int16MultiArray`) - Target RPM for each wheel.
  - `/ddsm115/brake` (`std_msgs/Bool`) - Brake command.
- **Publishes To:**
  - `/ddsm115/rpm_fb` (`std_msgs/Int16MultiArray`) - Actual RPM feedback from encoders.
  - `/mcu/yaw` (`std_msgs/Float32`) - Absolute yaw directly from the MCU.
  - `/wheel_odom` (`nav_msgs/Odometry`) - Pure wheel kinematics odometry calculated from encoder ticks.
  - `/sonar/*` (`std_msgs/Int32`) - Various ultrasonic sensor distance readings.

### `four_wheels_robot_node` (Package: `ddsm115_controller`)
*Translates high-level velocity commands (`/cmd_vel`) and joystick inputs into raw RPM commands.*
- **Subscribes To:**
  - `/cmd_vel` (`geometry_msgs/Twist`) - Target linear and angular velocity.
  - `/joy` (`sensor_msgs/Joy`) - Joystick inputs for manual control.
  - `/ddsm115/rpm_fb` (`std_msgs/Int16MultiArray`) - To calculate its own odometry (now legacy).
  - `/mcu/yaw` (`std_msgs/Float32`) - To calculate its own odometry (now legacy).
- **Publishes To:**
  - `/ddsm115/rpm_cmd` (`std_msgs/Int16MultiArray`) - The calculated RPMs to send to the wheels.
  - `/old_odom` (`nav_msgs/Odometry`) - Legacy odometry (remapped from `/odom` in launch file to avoid conflict with EKF).

### `rplidar_node` (Package: `rplidar_ros`)
*Driver for the RPLidar A1.*
- **Publishes To:**
  - `/scan` (`sensor_msgs/LaserScan`) - 2D LIDAR scan data.

### `tcp_bridge_node` (Package: `delivery_robot_core`)
*Minimalist custom protocol TCP bridge.*
- Acts as a bridge between the ROS 2 system and external TCP clients.

---

## 2. Localization & State Nodes

### `yaw_to_pose_node` (Package: `delivery_robot_core`)
*Lightweight translator that wraps the raw MCU yaw into a ROS standard Pose message for the EKF.*
- **Subscribes To:**
  - `/mcu/yaw` (`std_msgs/Float32`)
- **Publishes To:**
  - `/yaw_pose` (`geometry_msgs/PoseWithCovarianceStamped`) - Contains the absolute yaw as a Quaternion.

### `ekf_filter_node` (Package: `robot_localization`)
*Fuses pure wheel odometry and absolute IMU yaw into a highly accurate `/odom` state.*
- **Subscribes To:**
  - `/wheel_odom` (`nav_msgs/Odometry`) - Provided by `velocity_control_node` (Trusts X and Y velocity).
  - `/yaw_pose` (`geometry_msgs/PoseWithCovarianceStamped`) - Provided by `yaw_to_pose_node` (Trusts absolute yaw).
- **Publishes To:**
  - `/odom` (`nav_msgs/Odometry`) - The fused and filtered odometry.
  - `/tf` (`tf2_msgs/TFMessage`) - Broadcasts the `odom -> base_link` transform.

### `robot_state_publisher` (Package: `robot_state_publisher`)
*Broadcasts the static transforms of the robot based on `robot.urdf`.*
- **Publishes To:**
  - `/tf_static` (`tf2_msgs/TFMessage`) - Broadcasts `base_link -> laser_frame` and `base_link -> imu_link`.
