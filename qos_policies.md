# ROS 2 QoS Policies

ROS 2 uses DDS (Data Distribution Service) under the hood and with it comes powerful **QoS (Quality of Service)** settings to control how nodes communicate between each other
---

## 1. Reliability Policy

Controls how reliably messages are delivered from publishers to subscribers

| Setting         | Description                                                               |
|-----------------|---------------------------------------------------------------------------|
| `RELIABLE`      | Ensures delivery — retries if a message is lost                          |
| `BEST_EFFORT`   | Delivers what it can, no retries                                          |

**Usage Example**  
Use `RELIABLE` for commands or robot position  
Use `BEST_EFFORT` for non-critical sensor data like video streams

---

## 2. Durability Policy

Defines whether messages are saved and sent to late subscribers

| Setting             | Description                                                           |
|---------------------|-----------------------------------------------------------------------|
| `VOLATILE`          | Default. Only current subscribers receive messages                   |
| `TRANSIENT_LOCAL`   | Stores messages locally to send to future subscribers                |

**Usage Example**  
Use `TRANSIENT_LOCAL` for static info like map or robot URDF  
Use `VOLATILE` for live data like odometry or IMU

---

## 3. History Policy

Controls how many past messages are stored in the publisher or subscriber queue

| Setting         | Description                                                               |
|-----------------|---------------------------------------------------------------------------|
| `KEEP_LAST(n)`  | Keep only the last `n` messages (where `n = depth`)                      |
| `KEEP_ALL`      | Keep all messages in memory until delivered                              |

**Tip**  
`KEEP_LAST` is most commonly used with a specific depth (e.g., 10)

---

## 4. Deadline Policy

Specifies the expected interval between messages. If not received in time it's a missed deadline

| Setting Example | Description                                     |
|-----------------|-------------------------------------------------|
| `0.5s`          | Expect a message every 0.5 seconds              |

Used for system health checks or ensuring timely delivery

---

## 5. Liveliness Policy

Ensures the publisher is still alive and connected

| Setting             | Description                                                               |
|---------------------|---------------------------------------------------------------------------|
| `AUTOMATIC`         | System handles liveness. Used in most cases                             |
| `MANUAL_BY_TOPIC`   | Publisher manually asserts it's alive. Allows finer control             |

Helpful for detecting failures or disconnections

---

## 6. Lifespan Policy

Determines how long a message remains valid

| Setting Example | Description                            |
|-----------------|----------------------------------------|
| `2s`            | Message expires 2 seconds after publish|

Useful when data quickly becomes outdated like sensor readings

---

## Example usage

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
