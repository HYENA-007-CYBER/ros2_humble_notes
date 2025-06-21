#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleStopNode(Node):
    def __init__(self):
        super().__init__('obstacle_stop_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/four_wheel_bot/gazebo_ros_laser/out',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/four_wheel_bot/cmd_vel', 10)
        self.safe_distance = 0.5
        self.get_logger().info("Obstacle Stop Node Started")

    def lidar_callback(self, msg):
        # Filter out invalid ranges (e.g. inf) and find the minimum
        valid_ranges = [r for r in msg.ranges if r > 0.0]
        if not valid_ranges:
            return

        min_distance = min(valid_ranges)

        if min_distance < self.safe_distance:
            self.get_logger().warn(f"Obstacle too close: {min_distance:.2f} m. Stopping.")
            stop_msg = Twist()
            self.publisher.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


