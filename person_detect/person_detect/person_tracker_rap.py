import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2, time
import os
# os.environ['DISPLAY'] = ':0'
import csv
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from detections_msg.msg import Tracking

class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('person_tracker_node')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw/decompressed', self.listener_callback, qos)
        self.subscription2 = self.create_subscription(CompressedImage, '/oak/rgb/image_raw/dynamic/compressed', self.listener_callback2, qos)

        self.bridge = CvBridge()

        self.payload = 0.0

        self.publisher_ = self.create_publisher(Tracking, '/oak/yolo/detections', 10)

        self.declare_parameter("gt_folder", "/home/ros/rap/energy_ws/src/")
        dataset_path = self.get_parameter("gt_folder").get_parameter_value().string_value

        self.declare_parameter("target_gt_id", 3)
        self.target_gt_id = self.get_parameter("target_gt_id").get_parameter_value().integer_value
        
        gt_file = os.path.join(dataset_path, "gt.txt")

        # === Initialize model and tracker ===
        self.model = YOLO("yolov8n.pt").to("cuda")
        # self.model = YOLO("yolov8n.pt")
        print(f"Model device: {self.model.device}")
        self.tracker = DeepSort(max_age=30)

        self.declare_parameter("desired_id", 4)
        self.desired_id = self.get_parameter("desired_id").get_parameter_value().integer_value

        # === Load Ground Truth ===
        self.gt_dict = {}  # Format: {gt_frame_id: [(id, x, y, w, h), ...]}
   
        with open(gt_file, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                gt_frame_id = int(parts[0])
                track_id = int(parts[1])
                x, y, w, h = map(float, parts[2:6])
                mark = int(parts[6])
                visibility = float(parts[8])

                if track_id == self.target_gt_id and mark == 1:
                    if gt_frame_id not in self.gt_dict:
                        self.gt_dict[gt_frame_id] = []
                    self.gt_dict[gt_frame_id].append((track_id, x, y, w, h, visibility))

    # === IoU Function ===
    def compute_iou(self, boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[0] + boxA[2], boxB[0] + boxB[2])
        yB = min(boxA[1] + boxA[3], boxB[1] + boxB[3])
        interArea = max(0, xB - xA) * max(0, yB - yA)
        boxAArea = boxA[2] * boxA[3]
        boxBArea = boxB[2] * boxB[3]
        return interArea / float(boxAArea + boxBArea - interArea)
    
    def listener_callback(self, msg):

        t1 = time.time()
        
        self.get_logger().info("Received image")
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        frame_id_str = msg.header.frame_id
        img_name, comp, freq = frame_id_str.split(',')

        frame_id = int(os.path.splitext(img_name)[0])

        detections = []
        # At top of loop:
        start_time = time.time()

        # === Run YOLOv8n on the frame ===
        preds = self.model(frame, verbose=False)[0]
        for box in preds.boxes:
            cls_id = int(box.cls[0])
            if cls_id != 0:
                continue  # Only track 'person' class

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            bbox_w, bbox_h = x2 - x1, y2 - y1
            detections.append(([x1, y1, bbox_w, bbox_h], conf, None))

        # === Update tracker ===
        tracks = self.tracker.update_tracks(detections, frame=frame)

        tracked_box = [0, 0, 0, 0]

        iou = 0.0
        iou_vis = 0.0
        for track in tracks:
            if not track.is_confirmed():
                continue

            track_id = track.track_id

            if int(track_id) != self.desired_id:
                continue  

            l, t, r, b = track.to_ltrb()
            w, h = r - l, b - t
            tracked_box = [l, t, w, h]
            conf = 1.0

            # === Visualize ground truth for the target ID ===
            if frame_id in self.gt_dict:
                for gt_id, x, y, w, h, visible in self.gt_dict[frame_id]:
                    if gt_id == self.target_gt_id and tracked_box:
                        x1, y1, x2, y2 = int(x), int(y), int(x + w), int(y + h)

                        if tracked_box:
                            iou = self.compute_iou(tracked_box, [x, y, w, h])
                            iou_vis = min(iou / visible, 1.0)

        # At bottom of loop:
        elapsed = time.time() - start_time

        print(f"Frame {frame_id}")

        detections_msg = Tracking()
        detections_msg.track_id = self.desired_id
        detections_msg.target_gt_id = self.target_gt_id
        detections_msg.local_inference_time = elapsed
        detections_msg.header.stamp = msg.header.stamp
        detections_msg.header.frame_id = frame_id_str 
        detections_msg.accuracy = conf
        detections_msg.payload_bytes = self.payload
        detections_msg.confidence = conf
        detections_msg.transmission_time = t1 - (detections_msg.header.stamp.sec + (detections_msg.header.stamp.nanosec * 1e-9))
        detections_msg.iou = iou
        detections_msg.iou_visible = iou_vis
        detections_msg.bb_top_left_x = int(tracked_box[0])
        detections_msg.bb_top_left_y = int(tracked_box[1])
        detections_msg.bb_width = int(tracked_box[2])
        detections_msg.bb_height = int(tracked_box[3])

        self.publisher_.publish(detections_msg)

    def listener_callback2(self, msg):
        # self.get_logger().info("Received compressed image")
        self.payload = len(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()