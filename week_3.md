# 🤖 4-Wheeled Differential Drive Robot in ROS 2

This project simulates a 4-wheeled differential-drive robot using ROS 2 and Gazebo. The robot features a camera, LIDAR, and obstacle-avoidance behavior based on real-time sensor input.

---

## 📘 What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based language used to describe a robot's physical structure:
- Links (parts)
- Joints (connections)
- Visuals (for rendering)
- Collisions (for simulation physics)
- Inertials (mass and inertia)
- Plugins (for motion, sensors)

---

## 🧰 What is XACRO?

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
