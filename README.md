#  Week 1 - Basics of ROS 2

##  Objective
We need to learn about the basics of ROS2
- To create a **publisher node** that sends numbers
- To create a **subscriber node** that receives those numbers, calculates their square, and prints the result

---

##  Requirements
- **ROS 2 Version**: [ROS2-humble(jammy jellyfish)]
- **Operating System**: Ubuntu 22.04
- **Programming Language**: Python or c++
---

##  What I Learned



### Creating a ROS2 workspace
Workspace is a folder that contains ROS  packages and allows us to build and run them together<br>
We need to create a **src/(source folder)** and **(build/ and install/ and log/) folders** that are auto generated during the build process
- We need to create the packages inside the src/ folder
- We need to use the **colcon build** command to build all the packages in our workspace
- we need to source the **install/setup.bash** file so that the terminal can recognize the packages and can run the nodes

---


### Creating a ROS2 Package

A **ROS2 package** is the basic unit of organization in a ROS2 workspace. It contains everything needed for a module in a ROS system, such as:

- Node source files (Python or C++)
- Launch files
- Configuration files
- Dependencies
- Build and install instructions

All packages must be placed inside the `src/` folder of our ROS2 workspace. After creating packages, we need to  use `colcon build` to build the workspace and `source install/setup.bash` to use the packages in the terminal 



#### Creating a Python Package in ROS2

A **Python package** in ROS2 uses the `ament_python` build type and contains nodes written in Python

#####  Steps:

1. **Navigate to the workspace's `src/` folder**  
    ```bash
    cd ~/ros_ws/src/
    ```

2. **Create the Python package**  
    ```bash
    ros2 pkg create --build-type ament_python my_python_package --dependencies rclpy 
    ```

    This will:
    - Create a folder `my_python_package/`
    - Include `package.xml`, `setup.py`, and a `resource/` folder
    - Set up dependencies

3. **Add our Python node**
    Place your script in:  
    `my_python_package/my_python_package/my_node.py`

4. **Make the Python script executable**  
    ```bash
    chmod +x my_node.py
    ```

5. **Add entry points to `setup.py`**  
    Inside the `setup()` function, add:
    ```python
    entry_points={
        'console_scripts': [
            'my_node = my_python_package.my_node:main',
        ],
    },
    ```


#### Creating a C++ Package in ROS2

A **C++ package** in ROS2 uses the `ament_cmake` build type and contains nodes written in C++

#####  Steps:

1. **Navigate to our workspace's `src/` folder**  
    ```bash
    cd ~/ros_ws/src/
    ```

2. **Create the C++ package**  
    ```bash
    ros2 pkg create --build-type ament_cmake my_cpp_package --dependencies rclcpp 
    ```

    This will:
    - Create a folder `my_cpp_package/`
    - Include `package.xml`, `CMakeLists.txt`, and a `src/` folder
    - Set up dependencies

3. **Add our C++ node**
    Create the node file in:  
    `my_cpp_package/src/my_node.cpp`

4. **Edit `CMakeLists.txt` to build the node**
    Add the following:
    ```cmake
    add_executable(my_node src/my_node.cpp)
    ament_target_dependencies(my_node rclcpp )

    install(TARGETS
      my_node
      DESTINATION lib/${PROJECT_NAME})
    ```

---
### ROS2 Node
ROS 2 is built on the concept of **nodes**, which are independent processes that perform computation<br>
Nodes communicate with each other using **topics** through a **publish/subscribe** model



#### Basic Node Definition in Python

```python
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Here on u can have what u want to create
```
   - Node: The base class from rclpy used to define ROS2 nodes
   - `my_node_name`: The name by which the node is identified
#### Node commands:
   - rclpy.init() - Initializes the ROS2 Python interface
   - rclpy.shutdown() - Shuts it down
   - rclpy.spin(node) - Keeps the node running so it can respond to timers, subscriptions
   - destroy_node() - Cleans up resources used by the node before shutdown
#### After creating the node
- Add the node as entry point to setup.py





### ROS2 Topic

In ROS2, a **topic** is a communication channel that allows nodes to **publish** and **subscribe** messages

#### Concept:
- A **publisher** node sends messages to a named topic
- A **subscriber** node receives messages from that topic
- Nodes do not communicate directly with each other , they use topics as intermediaries

####  Characteristics:
- Topics are **unidirectional**: from publisher → subscriber(s)
- Messages must follow a defined **message type** (e.g., `std_msgs/String`, `sensor_msgs/Image`)
#### Add dependencies
- Based on the required message type add the dependency package to `package.xml`
- From that package retrieve the required datatype to use in node
  
#### Commands:
- To check details of a topic
```bash
ros2 topic info /topic_name
```
- To echo messages from a topic
```bash
ros2 topic echo /topic_name
```
- Use `rqt_graph` to check every publisher ,subscriber and topic present in our packages

## Publisher Node

A **Publisher Node** in ROS2 is a node that sends (publishes) messages to a **topic**

It acts as a data producer

#### Concept:
- The publisher node creates a **Publisher** object linked to a specific topic name and message type
- It then **publishes messages** on that topic using the `.publish()` method
- Any other node that subscribes to this topic will receive those messages
- Multiple subscribers can listen to the same topic





### **QUESTION PROBLEM** - Basic Structure OF Publisher node

```python
#!/usr/bin/env/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MyNode(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.publisher = self.create_publisher(Int32,'number',10)
        self.counter=1
        self.timer = self.create_timer(1.0 , self.publish_number)
        
    def publish_number(self):
        msg =Int32()
        msg.data =self.counter
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter +=1


def main(args=None):
    rclpy.init(args=args)
    node=MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ =='__main__':
    main()
```
#### Key Terms:
1. **create_publisher()**
   - Used to create a publisher that can send messages to a topic
    ```python
    self.publisher = self.create_publisher(Int32, 'number', 10)
    ```
     - Publishes **Int32** messages to the **number** topic
     - **Int32** - A standard message type provided by ROS2 (from std_msgs), representing a 32-bit signed integer
     - **10** - The queue size of the message

2. **create_timer()**
   - Sets up a periodic callback that executes a function at a fixed time interval
    ```python
    self.timer = self.create_timer(1.0, self.publish_number)
    ```
    - Calls `publish_number()` every 1 second
3. **publish()**
   - Used to send a message to the topic associated with the publisher
     ```python
     self.publisher.publish(msg)
     ```
4. **get_logger().info()**
   - Prints a log message to the terminal
     ```python
     self.get_logger().info(f'Publishing: {msg.data}')
     ```


## Subscriber Node

A **Subscriber Node** in ROS2 is a node that listens to messages from a **topic**.

It acts as a data consumer.

#### Concept:

- The subscriber node creates a **Subscriber** object linked to a specific topic name and message type  
- It then **receives messages** from that topic using a **callback function**  
- The callback function is automatically triggered whenever a new message is received  
- Multiple subscribers can listen to the same topic
- The subscriber must be spun using `rclpy.spin()` to keep listening for incoming messages
---


### **QUESTION PROBLEM** - Basic Structure OF Subscriber node

```python
#!/usr/bin/env/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MyNode(Node):
    def __init__(self):
        super().__init__('square_subscriber')
        self.subscriber=self.create_subscription(Int32,'number',self.listener_callback,10)
    def listener_callback(self,msg):
        square =msg.data**2
        self.get_logger().info(f'Received: {msg.data}, Square: {square}')



def main(args=None):
    rclpy.init(args=args)
    node=MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__=='__main__':
    main()
```
#### Key Terms:
1. **create_subscriber()**
   - Used to create a subscriber that listens for messages from a topic
    ```python
    self.subscriber = self.create_subscription(Int32, 'number', self.listener_callback, 10)
    ```
     - **self.listener_callback** - Callback function executed when a new message is received.
     - **Int32** - A standard message type provided by ROS2 (from std_msgs), representing a 32-bit signed integer
     - **10** - The queue size of the message(buffer size)

2. **listener_callback()**
   - This function is called every time a message is received on the subscribed topic
    ```python
    def listener_callback(self, msg):
    square = msg.data ** 2
    self.get_logger().info(f'Received: {msg.data}, Square: {square}')
    ```
    - Accesses the message data using msg.data
---


## To Run the Node
 ```bash
 cd/ros_ws
 colcon build --symlink-install
 source install/setup.bash
 ros2 run my_python_package my_node
 ```
  
---

## 1. What is DDS (Data Distribution Service)?

DDS stands for **Data Distribution Service** and it is the backbone of communication in ROS 2. It is not something developed exclusively for ROS ,it’s an industry-grade middleware standard designed to enable **high-performance, real-time, and scalable** data exchange in distributed systems

DDS operates on a **publish-subscribe architecture**, where nodes either publish data to topics or subscribe to data they are interested in. DDS handles the underlying complexity of connecting publishers to subscribers, even across different networks

###  Key Features of DDS in ROS 2:
- **Automatic Discovery**: Nodes can find each other without pre-configuration
- **Transport Flexibility**: Uses UDP by default, but can fall back on TCP. This makes it suitable for both low-latency local systems and less reliable wireless links
- **QoS Policies**: Quality of Service allows customization of how data is delivered (eg : reliability, history, deadlines)
- **Built-in Security and Scalability**: DDS is used in critical industries like aerospace and defense, so it’s inherently built to be robust and secure

ROS 2 abstracts most of DDS’s complexity, but its benefits  especially in multi-robot and distributed applications  are retained.

---

## 2. What is Peer-to-Peer Communication?

In ROS 2, communication happens in  **peer-to-peer (P2P)** method. This means:
- There is **no  master node** required for communication
- Every node in the system can independently discover and establish communication with other relevant nodes
- Once discovered, nodes connect **directly** to exchange messages, which minimizes latency and removes bottlenecks

### How Discovery Works:
DDS uses a mechanism known as **Simple Discovery Protocol (SDP)** for nodes to announce themselves and discover others.In large-scale systems, a **Discovery Server** can be used to optimize the discovery process, but even that does not act as a communication hub , just a directory to help reduce discovery traffic

This model is highly fault tolerant. If one node or even a group of nodes fail, the rest of the system continues to operate independently, which is good for robotics systems deployed in unpredictable environments

---

## 3. Why ROS 2 Moved Away from the ROS Master

In ROS 1, all communication was dependent on the **ROS Master**, a central process that kept track of publishers, subscribers, and services. While it was simple to implement and suitable for single-robot setups in controlled environments, it introduced several limitations

### Limitations of ROS 1 Master:
- **Single Point of Failure**: If the ROS Master crashed or was unreachable, nodes could not discover each other or maintain communication
- **Not Scalable**: Managing discovery and topic routing centrally became problematic in large  and distributed systems
- **Network Restrictions**: ROS Master assumes a shared network, which is difficult when working across multiple subnets or with robots deployed remotely
- **No QoS Support**: ROS 1 offered no native control over communication behavior (eg: retries, message delivery guarantees etc....)

---

## 4. How ROS 2 Improves Things with DDS

ROS 2  overcome these limitations by adopting DDS as its communication foundation. With DDS, ROS 2:
- **Removes the need for a central master**, making systems more resilient and fault tolerant
- **Allows scalable, distributed communication** across networks, which is ideal for real world multi-robot systems, IoT robotics, and industrial automation
- **Enables QoS settings**, allowing developers to tailor communication behavior based on reliability, latency, bandwidth, and other requirements
- **Supports dynamic discovery** and real-time data exchange

ROS 2 brings ROS into alignment with **modern distributed system practices**, using proven middleware that already powers mission-critical systems

---

## 5.ROS 1 vs ROS 2 Communication

| Feature                  | ROS 1                         | ROS 2 (with DDS)              |
|--------------------------|-------------------------------|-------------------------------|
| Discovery Mechanism      | Centralized (ROS Master)      | Distributed (via DDS)         |
| Communication            | Indirect via Master           | Direct peer-to-peer           |
| Failure Recovery         | Master failure = system crash | Resilient, no single point    |
| QoS Support              | None                          | Full QoS (reliability, etc...)|
| Multi-Robot Support      | Complex, limited               | Built-in and flexible        |
| Network Flexibility      | LAN-focused                   | Works across complex networks |
| Scalability              | Limited                       | High                          |

---

##  Conclusion

The transition from ROS 1 to ROS 2 represents a major architectural shift, focused on **robustness, flexibility, and scalability**. By using DDS and adopting a peer-to-peer model:
- ROS 2 eliminates the ROS Master as a bottleneck
- Enables dynamic, decentralized systems
- Supports a wider range of use cases — from simple robots to distributed, real-time, multi-agent systems

ROS 2 is better suited to meet the demands of **modern, real-world applications** in industry, research, and beyond


