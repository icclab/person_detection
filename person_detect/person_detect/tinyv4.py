import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, time
import os
import numpy as np

from ament_index_python.packages import get_package_share_directory

class YoloV4TinyNode(Node):
    def __init__(self):
        super().__init__('yolov4_tiny_node')
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.conf_threshold = 0.5

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

        # self.net = cv2.dnn.readNetFromDarknet('/home/icc-nano/energy_ws/src/test_yolo/testyolo/cfg/yolov4-tiny.cfg', '/home/icc-nano/energy_ws/src/test_yolo/testyolo/weights/yolov4-tiny.weights')

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
        fps = 1 / (end - start)
        self.get_logger().info(f"[YOLOv4-tiny] FPS: {fps:.2f}")

        # Extract detections
        height, width = frame.shape[:2]
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.conf_threshold:
                    class_name = self.class_names[class_id]
                    self.get_logger().info(f"Detected {class_name} with confidence {confidence:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloV4TinyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()