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