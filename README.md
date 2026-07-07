# ROS 2 Autonomous Delivery Robot

This repository contains the ROS 2 workspace for the Autonomous Delivery Robot. Below are the instructions and terminal commands required to get the system online and functional.

## Prerequisites

Ensure you have ROS 2 installed and properly sourced on your machine (e.g., `source /opt/ros/humble/setup.bash`), along with all necessary dependencies such as `nav2` and `slam_toolbox`.

## 1. Build the Workspace

Navigate to the root of the workspace and build the packages:

```bash
cd /home/ros2/works/robot_ws/ROS2-Autonomous-Robot
colcon build --symlink-install
```

## 2. Source the Workspace

After building, you must source the local environment in every new terminal you open before running any ROS 2 nodes or launch files:

```bash
source install/setup.bash
```

## 3. Launching the System

Depending on what you want to accomplish, you will need to open multiple terminals (remembering to source the workspace in each one). 

### Step 3a: Bringup Hardware (Required)
First, you must launch the hardware bringup to start the robot state publisher, LiDAR, and required communication bridge (TCP):

```bash
ros2 launch delivery_robot_bringup hardware_bringup.launch.py
```

### Step 3b: Choose your Operation Mode

After the hardware is up and running, choose one of the following modes:

**Option A: Autonomous Navigation & Mission Coordination**
To run the robot in navigation mode (using Nav2) along with the custom mission coordinator:

```bash
ros2 launch delivery_robot_bringup navigation_bringup.launch.py
```
*Note: This will use the pre-generated map located in the `maps/` directory.*

**Option B: Mapping (SLAM)**
If you need to create a new map of the environment using the SLAM Toolbox:

```bash
ros2 launch delivery_robot_bringup slam_bringup.launch.py
```
