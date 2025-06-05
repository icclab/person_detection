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
from detections_msg.msg import Tracking
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
import os

class YoloV8nNode(Node):
    def __init__(self):
        super().__init__('yolo_node_sub')
        
        self.subscription = self.create_subscription(Tracking, '/oak/yolo/detections', self.listener_callback, 10)

        self.start_time_str = time.strftime("%d-%m-%Y_%H-%M-%S")
        self.output_file = f"rap_tracker_{self.start_time_str}.csv"

        self.csvfile = open(self.output_file, "w", newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["unix_timestamp_sec", "frame_id", "tracker_id", "gt_id", "inference_time_sec", "bb_top_left_x", "bb_top_left_y", "bb_width", "bb_height", "conf", "IoU", "IoU_visible", "payload_bytes", "img_name", "freq", "compress"])
        self.csvfile.flush()

        self.get_logger().info(f"Logging to: {self.output_file}")
        
    def listener_callback(self, msg):

        msg_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        
        frame_id_str = msg.header.frame_id
        img_name, comp, freq = frame_id_str.split(',')
        frame_id = int(os.path.splitext(img_name)[0])
        
        self.writer.writerow([msg_time, frame_id, msg.track_id, msg.target_gt_id, msg.local_inference_time, msg.bb_top_left_x, msg.bb_top_left_y, msg.bb_width, msg.bb_height, msg.confidence, msg.iou, msg.iou_visible, msg.payload_bytes, img_name, freq, comp])

        self.csvfile.flush()

        self.get_logger().info("Received yolo tracker message")

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