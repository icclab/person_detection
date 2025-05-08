import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
import torch
import time
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from detections_msg.msg import Detections

class YoloV8nNode(Node):
    def __init__(self):
        super().__init__('yolov8n_node')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw/decompressed', self.listener_callback, qos)
        self.subscription2 = self.create_subscription(CompressedImage, '/oak/rgb/image_raw/dynamic/compressed', self.listener_callback2, qos)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.model.fuse()

        self.conf_threshold = 0.5
        self.payload = 0

        self.publisher_ = self.create_publisher(Detections, '/oak/yolo/detections', 10)
        
    def listener_callback(self, msg):
        self.get_logger().info("Received image")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        start = time.time()
        results = self.model.predict(frame, imgsz=640, device='cuda', verbose=False)
        end = time.time()
        conf = 0
        label = "None"
        # Process predictions
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                # self.get_logger().info(f"[YOLOv8n] CLASS ID: {cls_id}")
                if cls_id == 0:
                    conf = float(box.conf[0])
                    if conf > self.conf_threshold:
                        label = self.model.names[cls_id]
        
        self.get_logger().info(f"Detected {label} with confidence {conf:.2f}")

        detections_msg = Detections()
        detections_msg.class_id = label
        detections_msg.inference_time_s = end - start
        detections_msg.accuracy_percent = conf * 100
        detections_msg.payload_bytes = self.payload
        self.publisher_.publish(detections_msg)
        
        self.get_logger().info(f"Detected {label} with confidence {conf:.2f}")

    def listener_callback2(self, msg):
        # self.get_logger().info("Received compressed image")
        self.payload = len(msg.data)

    def __del__(self):
        if self.csvfile:
            self.csvfile.close()

def main(args=None):
    rclpy.init(args=args)
    node = YoloV8nNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
