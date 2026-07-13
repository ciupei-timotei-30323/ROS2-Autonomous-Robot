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
First, you must launch the hardware bringup to start the robot state publisher, LiDAR, and required communication bridge (TCP), as well as the new DDSM115 motor controllers:

> [!NOTE]
> Ensure your DDSM115 RS485 adapter is connected and recognized as `/dev/ttyUSB1`, and your RPLidar is on `/dev/ttyUSB0`. If they are reversed, you will need to swap the port names in `hardware_bringup.launch.py`. 
> 
> **Motor Initialization:** You do not need to manually initialize or configure the DDSM115 motors before starting the mapping or navigation modes. The `hardware_bringup` automatically detects the connected motors and places them into velocity control mode.

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
If you need to create a new map of the environment using the SLAM Toolbox, follow this multi-terminal workflow:

1. **Terminal 1 (Hardware)**: Launch the hardware bringup.
   ```bash
   ros2 launch delivery_robot_bringup hardware_bringup.launch.py
   ```
2. **Terminal 2 (SLAM)**: Launch the SLAM Toolbox to begin mapping.
   ```bash
   ros2 launch delivery_robot_bringup slam_bringup.launch.py
   ```
3. **Terminal 3 (Teleop)**: Open a new terminal, source the workspace, and run the teleop node to drive the robot around manually. Keep this terminal focused to use your keyboard.
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
4. **Terminal 4 (Waypoint Logger)**: Open a fourth terminal, source the workspace, and run the waypoint logger to safely save your coordinates into `Locations.json`. Whenever you park the robot in a spot you want to remember, type its name here!
   ```bash
   ros2 run delivery_robot_core waypoint_logger
   ```

Once you are done mapping, use the SLAM toolbox map saver panel in RViz to save the map!

## 4. Voice and TCP Commands

The `mission_coordinator_node` and `tcp_bridge_node` now provide a robust interface for sending commands via a TCP socket.

- **Manual Driving**: Send `Voice: forward`, `Voice: back`, `Voice: left`, or `Voice: right` to drive manually. The robot uses the LiDAR to detect obstacles and will automatically halt if anything is closer than **0.2m** in the direction of travel.
- **Autonomous Navigation**: Send `GO [destination]` (e.g., `GO Desk 1`). Coordinates are loaded dynamically from `src/delivery_robot_core/config/Locations.json` (meaning you can update coordinates on the fly without rebooting the node). If the robot gets stuck while navigating, it will alert the TCP client and attempt to re-plan every 5 seconds.
- **Emergency Stop**: Send `STOP` or `Voice: STOP` at any time to immediately interrupt any current navigation goals or manual movements and return the robot to an `IDLE` state. 

*Note: As the robot transitions between states (IDLE, MOVING_FORWARD, AUTONOMOUS_NAVIGATION, STUCK, etc.) or experiences events, it broadcasts alerts directly over the TCP connection (e.g., "Robot is stuck", "Robot arrived at destination").*
