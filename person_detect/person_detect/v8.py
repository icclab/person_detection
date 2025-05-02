import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
import torch

class YoloV8nNode(Node):
    def __init__(self):
        super().__init__('yolov8n_node')
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.listener_callback, qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.model.fuse()
        self.conf_threshold = 0.3

    def listener_callback(self, msg):
        self.get_logger().info("Received image")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        start = time.time()
        # results = self.model.predict(frame, imgsz=640, device='cpu', verbose=False)
        results = self.model.predict(frame, imgsz=640, device='cuda', verbose=False)
        end = time.time()
        fps = 1 / (end - start)
        self.get_logger().info(f"[YOLOv8n] FPS: {fps:.2f}")

        # torch.cuda.empty_cache()

        # Process predictions
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if conf > self.conf_threshold:
                    label = self.model.names[cls_id]
                    self.get_logger().info(f"Detected {label} with confidence {conf:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloV8nNode()
    rclpy.spin(node)
    rclpy.shutdown()
