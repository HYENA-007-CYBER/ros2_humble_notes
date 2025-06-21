#  4-Wheeled Differential Drive Robot in ROS 2

Need to simulate a 4-wheeled differential-drive robot using ROS 2 and Gazebo. The robot sholud feature a camera, LIDAR, and obstacle-avoidance behavior based on real-time sensor input

---
###  Package Structure

```
my_robot_description/
├── setup.py                     # Python entry point setup
├── setup.cfg                    # Metadata and options for setuptools
├── package.xml                  # ROS 2 package metadata
├── launch/
│   ├── gazebo.launch.py         # Launches Gazebo, the robot, and obstacle stop node
│   └── display.launch.py        # Launches RViz with the robot model
├── urdf/
│   └── four_wheel_bot.xacro     # URDF/XACRO model of the robot
├── worlds/
│   └── my_world.world           # Custom Gazebo world with an obstacle wall
├── my_robot_description/
│   ├── __init__.py              # Makes it a Python module
│   └── obstacle_stop.py         # ROS 2 node that stops the robot based on LIDAR
├── resource/
│   └── my_robot_description     # Required for ROS 2 indexing
```
---
##  What is URDF?

URDF stands for **Unified Robot Description Format**, and it’s basically an XML-based way to describe how a robot is built. It defines things like:
- The **links**, which are the individual parts of the robot (like the body and wheels)
- The **joints** that connect those parts
- **Visual** elements so it looks correct in simulation
- **Collision** boxes to interact properly with the environment
- And **inertial properties** like mass and inertia, which help simulate realistic physics

It also supports adding **plugins** for things like sensors and motion control

---

## What is XACRO?

While URDF is good, it can get repetitive. That’s where **XACRO** comes in — it’s like an advanced version of URDF that supports macros and parameters. It helped me avoid repeating the same code for similar parts (like the four wheels)

With XACRO, I could:
- Write **macros** to define a wheel once and reuse it
- Use **parameters** to quickly adjust dimensions and positions
- Do **math operations** directly in the file to make things cleaner and more modular

---

# XACRO Structure Overview
---
## Parameters

Defined at the top for easy configuration and reuse across the robot:

```xml
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="wheel_width" value="0.04"/>
<xacro:property name="chassis_length" value="0.4"/>
<xacro:property name="chassis_width" value="0.3"/>
<xacro:property name="chassis_height" value="0.1"/>
```

These allow easy tuning of the robot’s geometry and joint locations
---
## Base Link (Body)

The main chassis of the robot is represented using a box geometry. The `base_link` contains:

- **Visual & Collision** elements
- **Mass and Inertial** properties for dynamic simulation
- Positioned such that it sits above the ground

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
    </geometry>
  </visual>
  ...
  <inertial>
    <mass value="20"/>
    <inertia ixx="0.3" ... />
  </inertial>
</link>
```
---

## Wheels – Macros

Wheels are created using a `xacro:macro`, and reused for all four wheels (front/rear, left/right). Each wheel:

- Is a `cylinder` with correct rotation (`rpy="1.5708 0 0"`)
- Is connected to the base via a continuous joint
- Includes inertial and collision elements

```xml
<xacro:wheel name="front_left_wheel" x="..." y="..."/>
```

---

## Differential Drive Plugin

Allows the robot to move using the `cmd_vel` topic. Only rear wheels are driven , front wheels are for support

```xml
<plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
  <left_joint>rear_left_wheel_joint</left_joint>
  <right_joint>rear_right_wheel_joint</right_joint>
  <cmd_vel_topic>cmd_vel</cmd_vel_topic>
  <max_velocity>0.5</max_velocity>
</plugin>
```
Limits on speed and acceleration help avoid instability
---

## LIDAR Sensor

Mounted on top of the chassis. Simulates a 2D ray-based laser scanner:

```xml
<sensor type="ray" name="lidar_sensor">
  <update_rate>30</update_rate>
  <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so" />
</sensor>
```

Feeds distance data into `/gazebo_ros_laser/out` topic used by the obstacle stop node

---

##  Camera

Placed in front of the robot, mounted using a fixed joint. Outputs video feed via `/image_raw`:

```xml
<sensor type="camera" name="camera_sensor">
  <update_rate>30</update_rate>
  <plugin name="gazebo_ros_camera_controller" filename="libgazebo_ros_camera.so"/>
</sensor>
```

---

##  Inertial Configuration

To improve stability and avoid toppling:

- The mass is set to `20`
- Inertia matrix has uniform values
- Center of mass is lowered slightly with `<origin xyz="0 0 0.02"/>`

This improves simulation realism and reduces wobbling

---

## Total Overview

| Component   | Description |
|-------------|-------------|
| Base Link   | Main body with realistic mass and visuals |
| Wheels      | Cylindrical, macros reused for all 4 |
| Drive       | Differential drive plugin for motion |
| LIDAR       | Ray-based sensor for distance sensing |
| Camera      | Camera with live stream |
| Stability   | Improved using mass, inertia, and origin tuning |

---
# Teleoperation

To manually control the robot, we use the `teleop_twist_keyboard` ROS 2 package. This  send velocity commands (`/cmd_vel`) to the robot using the keyboard

## Installation

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

## Running Teleop

Make sure the robot and simulation are already running, then in a **new terminal**, source  ROS 2 workspace and run:

```bash
source ~/ros_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/four_wheel_bot/cmd_vel
```

This will start a keyboard interface. Use the keys below to control the robot:

```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i: move forward  
k: stop  
j/l: turn left/right  
,/: move backward  

Hold the key down for continuous motion.
```



# Obstacle Stop Node 

This Python node (`obstacle_stop.py`) implements a basic safety mechanism for the 4-wheeled differential drive robot in  Gazebo simulation

---

##  Overview

The **Obstacle Stop Node** listens to LIDAR sensor data and stops the robot if an obstacle is detected within a certain safe distance (default is 0.5 meters)

---
##  ROS 2 Interfaces

### Subscribed Topic
- **`/four_wheel_bot/gazebo_ros_laser/out`** (`sensor_msgs/msg/LaserScan`): LIDAR scan data from the robot

### Published Topic
- **`/four_wheel_bot/cmd_vel`** (`geometry_msgs/msg/Twist`): Velocity command to stop the robot

---

##  How It Works

1. **Initialization**:
   - A subscription to the LIDAR topic is created.
   - A publisher for the velocity command is created.
   - A safe distance is set to **0.5 meters**.

2. **Callback Function (`lidar_callback`)**:
   - Filters out invalid LIDAR values (e.g., 0.0 or `inf`)
   - Finds the minimum distance from the valid LIDAR readings
   - If the robot is too close to an obstacle (distance < 0.5m), it publishes a **zero velocity** command to stop

```python
stop_msg = Twist()  # zero linear and angular velocity
self.publisher.publish(stop_msg)
```
- Robot will stop even if it's being manually teleoperated
- Works best when robot has a forward-facing LIDAR
- We can increase stability by reducing speed or tuning mass/inertia in URDF

---
--- 

# HOW TO RUN 

Make sure your workspace is sourced and built:

```bash
colcon build --packages-select my_robot_description
source install/setup.bash
```

Then launch with your Gazebo world (including sensors):

```bash
ros2 launch my_robot_description gazebo.launch.py
```

For RVIZ visualisation launch with:

```bash
ros2 run my_robot_description display.launch
```
---


