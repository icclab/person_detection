import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
import torch
import csv
from datetime import datetime
from detections_msg.msg import Detections

class YoloV8nNode(Node):
    def __init__(self):
        super().__init__('yolo_node_sub')
        self.subscription = self.create_subscription(Detections, '/oak/yolo/detections', self.listener_callback, 10)
        
        self.start_time_str = time.strftime("%d-%m-%Y_%H-%M-%S")
        self.output_file = f"yolo_v4_{self.start_time_str}.csv"

        self.csvfile = open(self.output_file, "w", newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["unix_timestamp_sec", "class_id", "inference_time_sec", "accuracy_in_percent", "payload_bytes", "cuda"])
        self.csvfile.flush()

        self.get_logger().info(f"Logging to: {self.output_file}")
        
    def listener_callback(self, msg):
        self.get_logger().info("Received yolo detection message")

        self.get_logger().info(f"[YOLO] CLASS ID: {msg.class_id}")
        self.get_logger().info(f"[YOLO] INFERENCE TIME: {msg.inference_time_s:.2f} sec")
        self.get_logger().info(f"[YOLO] ACCURACY: {msg.accuracy_percent:.2f} %")
        self.get_logger().info(f"[YOLO] PAYLOAD SIZE: {msg.payload_bytes} bytes")
        self.get_logger().info(f"[YOLO] CUDA: {msg.cuda}")
        
        # self.writer.writerow([time.time(), msg.class_id, msg.inference_time_s, msg.accuracy_percent, msg.payload_bytes, msg.cuda])

        new_inference_time = time.time() - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) 

        self.writer.writerow([time.time(), msg.class_id, new_inference_time, msg.accuracy_percent, msg.payload_bytes, msg.cuda])
        
        self.csvfile.flush()

    def __del__(self):
        if self.csvfile:
            self.csvfile.close()


def main(args=None):
    rclpy.init(args=args)
    node = YoloV8nNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()