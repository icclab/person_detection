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
from detections_msg.msg import Detections

class YoloV8nNode(Node):
    def __init__(self):
        super().__init__('yolo_node_sub')
        self.subscription = self.create_subscription(Detections, '/oak/yolo/detections', self.listener_callback, 10)

        self.declare_parameter("output_prefix", "tegrastats_yolo")

        # Get prefix from param
        prefix = self.get_parameter("output_prefix").get_parameter_value().string_value

        self.declare_parameter("compress", 50)
        compression_level = self.get_parameter("compress").get_parameter_value().integer_value
        self.get_logger().info(f"Using compression level: {compression_level}")
        
        self.start_time_str = time.strftime("%d-%m-%Y_%H-%M-%S")
        self.output_file = f"{compression_level}_{prefix}_{self.start_time_str}.csv"

        self.csvfile = open(self.output_file, "w", newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["unix_timestamp_sec", "vdd_mW", "vdd_avg_mW", "energy_J", "energy_total_J", "class_id", "inference_time_sec", "accuracy_in_percent"])
        self.csvfile.flush()

        self.get_logger().info(f"Logging to: {self.output_file}")
        
        self.conf_threshold = 0.5
        self.tegrastats_logger = TegrastatsLogger()
        
    def listener_callback(self, msg):
        self.get_logger().info("Received yolo_v8n detection message")

        Detections_msg = Detections()
        Detections_msg.class_id = msg.class_id
        Detections_msg.inference_time_s = msg.inference_time_s
        Detections_msg.accuracy_percent = msg.accuracy_percent
        self.get_logger().info(f"[YOLOv8n] CLASS ID: {Detections_msg.class_id}")
        self.get_logger().info(f"[YOLOv8n] INFERENCE TIME: {Detections_msg.inference_time_s:.2f} sec")
        self.get_logger().info(f"[YOLOv8n] ACCURACY: {Detections_msg.accuracy_percent:.2f} %")

        unix_time, instantaneous_mW, average_mW, energy_J, energy_total_J = self.tegrastats_logger.log_tegrastats()

        self.writer.writerow([unix_time, instantaneous_mW, average_mW, energy_J, energy_total_J, Detections_msg.class_id, Detections_msg.inference_time_s, Detections_msg.accuracy_percent])
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
