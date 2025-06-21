#  4-Wheeled Differential Drive Robot in ROS 2

Need to simulate a 4-wheeled differential-drive robot using ROS 2 and Gazebo. The robot sholud feature a camera, LIDAR, and obstacle-avoidance behavior based on real-time sensor input

---
### 📁 Package Structure

```
my_robot_description/
├── CMakeLists.txt                # Build configuration
├── package.xml                  # Package metadata
├── launch/
│   └── gazebo.launch.py         # Launches Gazebo, the robot, and obstacle stop node
├── urdf/
│   └── four_wheel_bot.xacro     # URDF/XACRO model of the robot
├── worlds/
│   └── my_world.world           # Custom Gazebo world with an obstacle
├── my_robot_description/
│   ├── __init__.py              # Makes it a Python module
│   └── obstacle_stop.py         # ROS 2 node that stops the robot based on LIDAR
├── resource/
│   └── my_robot_description     # Required for ROS 2 indexing
```
---
##  What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based language used to describe a robot's physical structure:
- Links (parts)
- Joints (connections)
- Visuals (for rendering)
- Collisions (for simulation physics)
- Inertials (mass and inertia)
- Plugins (for motion, sensors)

---

## What is XACRO?

**XACRO** (XML Macros) is a more flexible version of URDF that allows reuse through:
- Macros (e.g., defining a wheel once)
- Parameters (to control size, placement, etc.)
- Math expressions (for cleaner calculations)

---

## 🛠️ XACRO Structure Overview

### 🔢 Parameters

At the top of the XACRO:

```xml
<xacro:property name="chassis_length" value="0.4"/>
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="chassis_height" value="0.1"/>
