# pip3 install torch torchvision torchaudio
# pip3 install jinja2 typeguard
# pip install -U openmim
# mim install mmengine
# mim install "mmcv==2.1.0" --use-pep517
# cd
# git clone https://github.com/open-mmlab/mmdetection.git
# cd mmdetection
# pip install -v -e .

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2, time
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from detections_msg.msg import Detections
from rclpy.executors import MultiThreadedExecutor
from mmdet.apis import DetInferencer

class DinoDetectorNode(Node):
    def __init__(self):
        super().__init__('dino_detector_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw/decompressed', self.listener_callback, qos)
        self.subscription2 = self.create_subscription(CompressedImage, '/oak/rgb/image_raw/dynamic/compressed', self.listener_callback2, qos)

        self.bridge = CvBridge()
        self.payload = 0

        self.publisher_ = self.create_publisher(Detections, '/oak/yolo/detections', 10)

        # Initialize MMDetection DINO
        config_path = '/home/ros/mmdetection/configs/dino/dino-5scale_swin-l_8xb2-36e_coco.py'
        checkpoint = '/home/ros/mmdetection/checkpoint/dino-5scale_swin-l_8xb2-36e_coco-5486e051.pth'
        self.inferencer = DetInferencer(model=config_path, weights=checkpoint, device='cuda:0')

        self.tp = 0
        self.tn = 0
        self.fp = 0
        self.fn = 0

    def listener_callback(self, msg):
        t1 = time.time()
        # self.get_logger().info("Received image")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        start = time.time()

        result = self.inferencer(frame)
        end = time.time()

        self.get_logger().info(f"Inference time: {end - start:.4f} seconds")

        labels = result["predictions"][0]["labels"]
        scores = result["predictions"][0]["scores"]
        label_ids = [int(l) for l in labels]

        # self.get_logger().info(f"Detected labels: {label_ids}")

        max_conf = 0.0
        person_confidences = [scores[i] for i, label in enumerate(label_ids) if label == 0]

        if person_confidences:
            max_conf = float(max(person_confidences))
            self.get_logger().info(f"'person' detected with max confidence: {max_conf:.4f}")
            person_bool = 1
        else:
            self.get_logger().info("No 'person' detected.")
            person_bool = 0

        detections_msg = Detections()
        detections_msg.header.stamp = msg.header.stamp
        detections_msg.class_id = "person" if person_bool else "None"
        detections_msg.accuracy_percent = max_conf * 100
        detections_msg.payload_bytes = self.payload
        detections_msg.cuda = True
        detections_msg.person_bool = person_bool
        detections_msg.local_inference_time = end - start
        detections_msg.transmission_time = t1 - (detections_msg.header.stamp.sec + (detections_msg.header.stamp.nanosec * 1e-9))

        frame_id_str = msg.header.frame_id
        gt, detections_msg.header.frame_id, comp, fq = frame_id_str.split(',')

        detections_msg.ground_truth = int(gt)
        detections_msg.compress = int(comp)
        detections_msg.freq = float(fq)

        if int(gt) == 1 and person_bool == 1:
            self.tp += 1
        elif int(gt) == 1 and person_bool == 0:
            self.fn += 1
        elif int(gt) == 0 and person_bool == 0:
            self.tn += 1
        elif int(gt) == 0 and person_bool == 1:
            self.fp += 1

        detections_msg.tp = self.tp
        detections_msg.tn = self.tn
        detections_msg.fp = self.fp
        detections_msg.fn = self.fn

        if self.tp > 0:
            detections_msg.precision = self.tp / (self.tp + self.fp)
            detections_msg.recall = self.tp / (self.tp + self.fn)
            self.get_logger().info(f"Precision: {detections_msg.precision:.4f}, Recall: {detections_msg.recall:.4f}")

        self.publisher_.publish(detections_msg)
        self.get_logger().info(f"Detected {'person' if person_bool else 'no person'} with max. confidence {max_conf:.2f}")

    def listener_callback2(self, msg):
        self.payload = len(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = DinoDetectorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
