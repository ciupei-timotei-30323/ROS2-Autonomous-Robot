# DDSM115 ROS2 controller

This project is based from my previous [Python DDSM115 library](https://github.com/rasheeddo/ddsm115_python), and this project could make a control of DDSM115 a lot easier in ROS2 environment.

## Deps

- `pip3 install crcmod pyserial`

## Set Motor ID

You will need to connect only one motor on the RS485 bus, then run,

```sh
ros2 run ddms115_controller set_motor_id
## Then input the ID number as you want, then the node will be shutdown.
```

## Check Online Motor ID

You can plug all of post set ID motor to the RS485 bus, then run,

```sh
ros2 run ddsm115_controller check_motor_id
## It will return a list of online ID in the bus, then the node will be shutdown.
```

## Velocity Control

Once all the motor ID is setup and confirm, you can run 

```sh
ros2 run ddsm115_controller velocity_control
## This node will check all the online ID on the bus, 
## then you can control and read data fom each motor from ROS2 topics.
```

### Subscriber topics

- `/ddsm115/rpm_cmd` : as *std_msgs/msg/Int16MultiArray*, the node is subscribe on this command, user can publish as, e.g. `[id_1_rpm,  id_2_rpm, id3_rpm, ...]` . If you only have 1 motor as ID 1 then just publish as  `[id_1_rpm]`, if there are two ID as 1,2 then just publish as `[id_1_rpm, id_2_rpm]`, if you have ID 1 and 3 but no 2, then just publish as `[id_1_rpm, 0, id_3_rpm]`. So the rpm command of each ID must be on the correct index in the command list.

- `/ddsm115/brake` : as *std_msgs/msg/Bool*, the node is subscribe on this brake flag, if user sent `True` then all the motors will be braked, as default all the motors are free to rotate but with some reaction force from magnetic field. But with brake as `True` then it will hold the current position in place. And if publish as `False` to release the brake of all motors.

### Publisher topics

- `/ddsm115/rpm_fb` : as *std_msgs/msg/Int16MultiArray*, the node will be publishing rpm feedback of all motors as a list, so index 1 as ID1, index 2 as ID2, and so on.

- `/ddsm115/cur_fb` : as *std_msgs/msg/Float32MultiArray*, the node will be publishing current feedback in amperes unit  of all motors, the list index is similar to rpm feedback.

- `/ddsm115/temp_fb` : as *std_msgs/msg/Int8MultiArray*, the node will be publishing temperature feedback of all motors as a list.

- `/ddsm115/error` : as *std_msgs/msg/Int8MultiArray*, the node will be publishing error feedback of all motors as a list.
```
1 : for sensor error
2 : for over current 
4 : for phase over error
8 : for stall error
16 : for troubleshoot error
```

- `/ddsm115/online_id` : as *std_msgs/msg/Int8MultiArray*, the node will be publishing online motor ID of all motors as a list.

## Two Wheels Robot

There is a node to make your robot quickly start as a mobile robot, so make sure to have left wheel motor setup as ID 1 and right wheel motor setup as ID 2.

```sh
ros2 run ddsm115_controller two_wheels_robot
## OR
ros2 run ddsm115_controller two_wheels_robot --ros-args -p pub_tf:=True
## if you need an odom->base_link TF published.
```

This node is subscribing on `/cmd_vel` topic then convert that to `/ddsm115/rpm_cmd` to drive motors, and also publishing an `/odom` odometry topic which calculated from `/ddsm115/rpm_fb` wheel's speed. 

You can use `rqt_robot_steering` to manually drive robot OR you can use `ros2 run joy joy_node` to publish joystick topic to control the robot from joystick.

You can change a parameter of `wheel_base` to match with your robot wheel's base.