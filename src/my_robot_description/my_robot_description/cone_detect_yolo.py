import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

class YOLOConeDetector(Node):
    def __init__(self):
        super().__init__('yolo_cone_detector')
        self.bridge = CvBridge()

        # Subscribe to camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/four_wheel_bot/camera_sensor/image_raw',
            self.image_callback,
            10
        )

        # Publisher to indicate detection
        self.pub = self.create_publisher(Bool, 'cone_detected', 10)

        # Load YOLOv5 model
        self.get_logger().info("Loading YOLOv5 model...")
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.conf = 0.4  # confidence threshold
        self.get_logger().info("YOLOv5 Cone Detector started.")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run inference
        results = self.model(cv_image)

        detected = False
        for *xyxy, conf, cls in results.xyxy[0]:
            label = self.model.names[int(cls)].lower()
            if "cone" in label or "traffic" in label:
                detected = True
                x1, y1, x2, y2 = map(int, xyxy)

                # Draw bounding box
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, f'{label} {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Log to terminal
                self.get_logger().info(
                    f"Detection: {label} with {conf:.2f} confidence at [{x1},{y1},{x2},{y2}]"
                )
                break

        # Publish detection result
        self.pub.publish(Bool(data=detected))

        if not detected:
            self.get_logger().info("No cone detected.")

        # Show result
        cv2.imshow("YOLO Cone Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
