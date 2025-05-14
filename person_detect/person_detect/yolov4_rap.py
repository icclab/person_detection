import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2, time
import os
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from detections_msg.msg import Detections
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

class YoloV4Node(Node):
    def __init__(self):
        super().__init__('yolov4_node')
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw/decompressed', self.listener_callback, qos)
        self.subscription2 = self.create_subscription(CompressedImage, '/oak/rgb/image_raw/dynamic/compressed', self.listener_callback2, qos)
        self.bridge = CvBridge()
        self.conf_threshold = 0.5
        self.payload = 0

        self.publisher_ = self.create_publisher(Detections, '/oak/yolo/detections', 10)

        cfg_path = os.path.join(
            get_package_share_directory('person_detect'),
            'launch',
            'yolov4.cfg'
        )
        weights_path = os.path.join(
            get_package_share_directory('person_detect'),
            'launch',
            'yolov4.weights'
        )

        self.net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
        
        self.declare_parameter("use_cuda", True)

        self.use_cuda = self.get_parameter("use_cuda").get_parameter_value().bool_value
        
        if self.use_cuda:       
            self.get_logger().info("Using CUDA for inference")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            # self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA_FP16)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        else:   
            self.get_logger().info("Using CPU for inference")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            # self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU_FP16)
        
        names_path = os.path.join(
            get_package_share_directory('person_detect'),
            'config',
            'coco.names'
        )

        with open(names_path, "r") as f:
            self.class_names = f.read().strip().split("\n")

    def listener_callback(self, msg):
        
        self.get_logger().info("Received image")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())

        class_name = "None"
        conf = 0.0
        max_conf = 0.0
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
                        if conf > max_conf:
                            max_conf = conf 

                        class_name = self.class_names[class_id]

        detections_msg = Detections()
        detections_msg.header.stamp = msg.header.stamp
        detections_msg.class_id = class_name
        detections_msg.accuracy_percent = max_conf * 100
        detections_msg.payload_bytes = self.payload
        detections_msg.cuda = self.use_cuda
        self.publisher_.publish(detections_msg)

        self.get_logger().info(f"Detected {class_name} with max. confidence {max_conf:.2f}")

    def listener_callback2(self, msg):
        # self.get_logger().info("Received compressed image")
        self.payload = len(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = YoloV4Node()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()