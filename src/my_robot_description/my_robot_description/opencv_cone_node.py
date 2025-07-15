import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class ConeDetector(Node):
    def __init__(self):
        super().__init__('cone_detector')
        self.bridge = CvBridge()

        # Subscribe to Gazebo camera image
        self.create_subscription(
            Image,
            '/four_wheel_bot/camera_sensor/image_raw',
            self.image_callback,
            10
        )

        # Publisher to notify detection
        self.detection_pub = self.create_publisher(Bool, 'cone_detected', 10)

        self.get_logger().info("Cone Detector node started")

    def image_callback(self, msg):
        # Convert to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define orange HSV range
        lower_orange = np.array([5, 100, 100])
        upper_orange = np.array([20, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:
                x, y, w, h = cv2.boundingRect(cnt)
                center_x = x + w // 2
                center_y = y + h // 2

                # Estimate distance from size
                distance = round(200.0 / h, 2) if h > 0 else 0

                # Logging to terminal
                self.get_logger().info(f"Orange cone detected at approx {distance} meters")

                # Draw box and label
                label = f"Orange Cone | {distance}m"
                if distance < 1.0:
                    label += " CLOSE"

                text_y = max(y - 10, 20)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, label, (x, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 0), 4)
                cv2.putText(frame, label, (x, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2)

                # Publish detection result
                self.detection_pub.publish(Bool(data=True))
                detected = True
                break  # stop after first match

        if not detected:
            self.detection_pub.publish(Bool(data=False))
            self.get_logger().info("No cone detected.")

        # Display output windows
        cv2.imshow("Camera View", frame)
        cv2.imshow("Orange Mask", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
