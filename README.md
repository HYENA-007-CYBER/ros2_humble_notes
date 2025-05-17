#  Week 1 - Basics of ROS 2

##  Objective
We need to learn about the basics of ROS2
- To create a **publisher node** that sends numbers
- To create a **subscriber node** that receives those integers, calculates their square, and prints the result

---

##  Requirements
- **ROS 2 Version**: [ROS2-humble(jammy jellyfish)]
- **Operating System**: Ubuntu 22.04
- **Programming Language**: Python
---

##  What I Learned

###  ROS2 Framework
- ROS 2 is built on the concept of **nodes**, which are independent processes that perform computation
- Nodes communicate with each other using **topics** through a **publish/subscribe** model

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

### ROS2 Topic

In ROS2, a **topic** is a communication channel that allows nodes to **publish** and **subscribe** messages

#### Concept:
- A **publisher** node sends messages to a named topic
- A **subscriber** node receives messages from that topic
- Nodes do not communicate directly with each other , they use topics as intermediaries

####  Characteristics:
- Topics are **unidirectional**: from publisher → subscriber(s)
- Messages must follow a defined **message type** (e.g., `std_msgs/String`, `sensor_msgs/Image`)
  
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
5. - rclpy.init() - Initializes the ROS2 Python interface
   - rclpy.shutdown() - Shuts it down
   - rclpy.spin(node) - Keeps the node running so it can respond to timers, subscriptions
   - destroy_node() - Cleans up resources used by the node before shutdown

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
