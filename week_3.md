#  4-Wheeled Differential Drive Robot in ROS 2

Need to simulate a 4-wheeled differential-drive robot using ROS 2 and Gazebo. The robot sholud feature a camera, LIDAR, and obstacle-avoidance behavior based on real-time sensor input

---
### 📁 Package Structure

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
- Write **macros** to define a wheel once and reuse it,
- Use **parameters** to quickly adjust dimensions and positions,
- Do **math operations** directly in the file to make things cleaner and more modular.

---

## 🛠️ XACRO Structure Overview

### 🔢 Parameters

At the top of the XACRO:

```xml
<xacro:property name="chassis_length" value="0.4"/>
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="chassis_height" value="0.1"/>
