import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
import torch
from .log_tegrastats import TegrastatsLogger
import csv
from datetime import datetime

class YoloV8nNode(Node):
    def __init__(self):
        super().__init__('yolov8n_node')
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.listener_callback, qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.model.fuse()
        self.cuda = True

        self.start_time_str = time.strftime("%d-%m-%Y_%H-%M-%S")
        self.output_file = f"tegrastats_yolo_v8_{self.start_time_str}.csv"

        self.csvfile = open(self.output_file, "w", newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["unix_timestamp_sec", "vdd_mW", "vdd_avg_mW", "energy_J", "energy_total_J", "class_id", "inference_time_sec", "accuracy_in_percent", "msg_payload_size"])
        self.csvfile.flush()
        
        self.conf_threshold = 0.5
        self.tegrastats_logger = TegrastatsLogger()
        
    def listener_callback(self, msg):
        self.get_logger().info("Received image")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        start = time.time()
        
        if self.cuda:
            self.get_logger().info("Using CUDA for inference")
            results = self.model.predict(frame, imgsz=640, device='cuda', verbose=False)

        else:
            self.get_logger().info("Using CPU for inference")
            results = self.model.predict(frame, imgsz=640, device='cpu', verbose=False)
        
        end = time.time()
        
        # fps = 1 / (end - start)
        # self.get_logger().info(f"[YOLOv8n] FPS: {fps:.2f}")

        # torch.cuda.empty_cache()

        conf = 0
        label = None
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

        unix_time, instantaneous_mW, average_mW, energy_J, energy_total_J = self.tegrastats_logger.log_tegrastats()

        self.writer.writerow([unix_time, instantaneous_mW, average_mW, energy_J, energy_total_J, label, end - start, conf * 100, len(msg.data)])
        self.csvfile.flush()

    def __del__(self):
        if self.csvfile:
            self.csvfile.close()
        self.tegrastats_logger.close()


def main(args=None):
    rclpy.init(args=args)
    node = YoloV8nNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
