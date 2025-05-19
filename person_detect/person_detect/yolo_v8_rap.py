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

        self.tp = 0
        self.tn = 0
        self.fp = 0
        self.fn = 0

        self.conf_threshold = 0.5
        self.payload = 0
        
        self.declare_parameter("use_cuda", True)

        self.use_cuda = self.get_parameter("use_cuda").get_parameter_value().bool_value

        if self.use_cuda:       
            self.device = 'cuda' 
            self.get_logger().info("Using CUDA for inference")

        else:
            self.device = 'cpu'   
            self.get_logger().info("Using CPU for inference")

        self.publisher_ = self.create_publisher(Detections, '/oak/yolo/detections', 10)
        
    def listener_callback(self, msg):

        t1 = time.time()
        
        self.get_logger().info("Received image")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        start = time.time()
        results = self.model.predict(frame, imgsz=640, device=self.device, verbose=False)
        end = time.time()
        conf = 0
        label = "None"
        # Process predictions
        height, width = frame.shape[:2]
        
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                # self.get_logger().info(f"[YOLOv8n] CLASS ID: {cls_id}")
                if cls_id == 0:
                    conf = float(box.conf[0])
                    if conf > self.conf_threshold:
                        label = self.model.names[cls_id]

        person_bool = 0

        if conf > 0.0 and label == "person":
            person_bool = 1
        else:
            person_bool = 0
        
        detections_msg = Detections()
        detections_msg.header.stamp = msg.header.stamp
        detections_msg.class_id = label
        detections_msg.accuracy_percent = conf * 100
        detections_msg.payload_bytes = self.payload
        detections_msg.cuda = self.use_cuda
        detections_msg.person_bool = person_bool
        detections_msg.local_inference_time = end - start
        detections_msg.transmission_time = t1 - (detections_msg.header.stamp.sec + (detections_msg.header.stamp.nanosec * 1e-9))
        frame_id_str = msg.header.frame_id
        gt, detections_msg.header.frame_id, comp, fq = frame_id_str.split(',')

        detections_msg.ground_truth = int(gt)
        detections_msg.compress = int(comp)
        detections_msg.freq = float(fq)

        if int(gt) == 1 and int(person_bool) == 1:
            self.tp += 1

        elif int(gt) == 1 and int(person_bool) == 0:
            self.fn += 1
        
        elif int(gt) == 0 and int(person_bool) == 0:
            self.tn += 1

        elif int(gt) == 0 and int(person_bool) == 1:
            self.fp += 1

        detections_msg.tp = self.tp
        detections_msg.tn = self.tn
        detections_msg.fp = self.fp
        detections_msg.fn = self.fn

        if self.tp > 0:
            detections_msg.precision = self.tp / (self.tp + self.fp)
            detections_msg.recall = self.tp / (self.tp + self.fn)

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
