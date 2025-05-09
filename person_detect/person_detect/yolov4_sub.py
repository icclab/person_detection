import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import time
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
import torch
import csv
from datetime import datetime
from detections_msg.msg import Detections
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

class YoloV8nNode(Node):
    def __init__(self):
        super().__init__('yolo_node_sub')
        
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(Detections, '/oak/yolo/detections', self.listener_callback, 10)
        self.subscription2 = self.create_subscription(CompressedImage, '/oak/rgb/image_raw/dynamic/compressed', self.listener_callback2, qos)
        
        self.start_time_str = time.strftime("%d-%m-%Y_%H-%M-%S")
        self.output_file = f"yolo_v4_{self.start_time_str}.csv"

        self.csvfile = open(self.output_file, "w", newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["unix_timestamp_sec", "class_id", "inference_time_sec", "accuracy_in_percent", "payload_bytes", "cuda"])
        self.csvfile.flush()

        self.get_logger().info(f"Logging to: {self.output_file}")

        self.time_output_file = f"time_yolo_v4_{self.start_time_str}.csv"

        self.time_file = open(self.time_output_file, "w", newline='')
        self.time_writer = csv.writer(self.time_file)
        self.time_writer.writerow(["image_raw_compressed_timestamp_sec"])
        self.time_file.flush()

        self.get_logger().info(f"Logging to: {self.time_output_file}")
        
    def listener_callback2(self, msg):
        
        self.time_writer.writerow([time.time()])
        
    def listener_callback(self, msg):

        self.writer.writerow([time.time(), msg.class_id, msg.accuracy_percent, msg.payload_bytes, msg.cuda])

        self.csvfile.flush()

        self.get_logger().info("Received yolo detection message")

        self.get_logger().info(f"[YOLO] CLASS ID: {msg.class_id}")
        self.get_logger().info(f"[YOLO] ACCURACY: {msg.accuracy_percent:.2f} %")
        self.get_logger().info(f"[YOLO] PAYLOAD SIZE: {msg.payload_bytes} bytes")
        self.get_logger().info(f"[YOLO] CUDA: {msg.cuda}")

    def __del__(self):
        if self.csvfile:
            self.csvfile.close()
        if self.time_file:
            self.time_file.close()

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