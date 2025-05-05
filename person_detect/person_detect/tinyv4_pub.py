import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, time
import os
import csv
import numpy as np
from detections_msg.msg import Detections
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

class YoloV4TinyNode(Node):
    def __init__(self):
        super().__init__('yolov4_tiny_node_pub')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw/decompressed', self.listener_callback, qos)
        self.bridge = CvBridge()
        self.conf_threshold = 0.5

        self.publisher_ = self.create_publisher(Detections, '/oak/yolo/detections', 10)

        cfg_path = os.path.join(
            get_package_share_directory('person_detect'),
            'launch',
            'yolov4-tiny.cfg'
        )
        weights_path = os.path.join(
            get_package_share_directory('person_detect'),
            'launch',
            'yolov4-tiny.weights'
        )

        self.net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
        
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        
        names_path = os.path.join(
            get_package_share_directory('person_detect'),
            'config',
            'coco.names'
        )

        with open(names_path, "r") as f:
            self.class_names = f.read().strip().split("\n")

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        start = time.time()
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        end = time.time()
        # fps = 1 / (end - start)
        # self.get_logger().info(f"[YOLOv4-tiny] FPS: {fps:.2f}")

        class_name = None
        conf = 0
        # Extract detections
        height, width = frame.shape[:2]
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                if class_id == 0:
                    confidence = scores[class_id]
                    if confidence > self.conf_threshold:
                        conf = float(confidence)
                        class_name = self.class_names[class_id]

                        detections_msg = Detections()
                        detections_msg.class_id = class_name
                        detections_msg.inference_time_s = end - start
                        detections_msg.accuracy_percent = conf * 100
                        self.publisher_.publish(detections_msg)

                        self.get_logger().info(f"Detected {class_name} with confidence {confidence:.2f}")

    def __del__(self):
        if self.csvfile:
            self.csvfile.close()
        self.tegrastats_logger.close()

def main(args=None):
    rclpy.init(args=args)
    node = YoloV4TinyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()