# ROS 2 Autonomous Delivery Robot

This repository contains the ROS 2 workspace for the Autonomous Delivery Robot. Below are the instructions and terminal commands required to get the system online and functional.

## Prerequisites

Ensure you have ROS 2 installed and properly sourced on your machine (e.g., `source /opt/ros/humble/setup.bash`), along with all necessary dependencies such as `nav2` and `slam_toolbox`.

## Hardware Setup (First Time Only)

If you are setting up brand new DDSM115 motors out of the box, they all come with a default ID of `1`. You must configure them with unique IDs before plugging them all into the robot. **You only need to do this once.**

The robot expects the following ID mapping:
- **ID 1:** Front Left
- **ID 2:** Front Right
- **ID 3:** Rear Left
- **ID 4:** Rear Right

**Configuration Steps:**
1. Connect **only one motor** to your RS485 driver board. Supply power to it.
2. Run the setup script:
   ```bash
   python3 src/ddsm115_controller/ddsm115_controller/set_motor_id.py
   ```
3. Type the desired ID (e.g., `1` for Front Left) when prompted and press Enter.
4. **Power cycle** the motor so the new ID takes effect.
5. Repeat steps 1-4 for the remaining motors, assigning IDs 2, 3, and 4.

Once all four motors have their unique IDs assigned, you can connect them all in parallel to the RS485 bus.

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
> Ensure your STM32 driver board is properly wired to the Raspberry Pi's SPI pins (CE0, SCLK, MISO, MOSI) with a common ground, and your RPLidar is connected to `/dev/ttyUSB0`.
> 
> **Motor Control:** As long as your motors have been configured with unique IDs 1-4, the `hardware_bringup` will automatically stream velocities to them via the STM32 over SPI.

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
