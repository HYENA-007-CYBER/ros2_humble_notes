#!/usr/bin/env/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Int32

class MyNode(Node):
    def __init__(self):
        super().__init__('square_subscriber')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.subscriber = self.create_subscription(
            Int32,
            'number',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        square = msg.data ** 2
        self.get_logger().info(f'Received: {msg.data}, Square: {square}')


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
