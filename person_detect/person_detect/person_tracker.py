import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, time
import os
# os.environ['DISPLAY'] = ':0'
import csv
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory

class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('person_tracker_node')
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()

        self.start_time_str = time.strftime("%d-%m-%Y_%H-%M-%S")
        self.output_file = f"tracker_orin_{self.start_time_str}.csv"

        self.csvfile = open(self.output_file, "w", newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["unix_timestamp_sec", "frame_id", "tracker_id", "gt_id", "inference_time_sec", "bb_top_left_x", "bb_top_left_y", "bb_width", "bb_height", "conf", "IoU", "IoU_visible", "payload_bytes", "img_name", "freq", "compress"])
        self.csvfile.flush()
        self.get_logger().info(f"Logging to: {self.output_file}")

        self.declare_parameter("gt_folder", "/home/icc-nano/energy_ws/src/MOT20-01/")
        dataset_path = self.get_parameter("gt_folder").get_parameter_value().string_value
    
        self.output_txt = f"yolov8n_deepsort_{self.start_time_str}.txt"

        self.csvfile2 = open(self.output_txt, "w", newline='')
        self.writer2 = csv.writer(self.csvfile2)

        self.declare_parameter("target_gt_id", 3)
        self.target_gt_id = self.get_parameter("target_gt_id").get_parameter_value().integer_value
        
        gt_file = os.path.join(dataset_path, "gt", "gt.txt")

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

            self.writer2.writerow([frame_id, track_id, int(l), int(t), int(w), int(h), conf, -1, -1, -1])
            self.csvfile2.flush()

            # === Visualize ground truth for the target ID ===
            if frame_id in self.gt_dict:
                for gt_id, x, y, w, h, visible in self.gt_dict[frame_id]:
                    if gt_id == self.target_gt_id and tracked_box:
                        x1, y1, x2, y2 = int(x), int(y), int(x + w), int(y + h)
                        # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        # cv2.putText(frame, f"GT ID {gt_id}", (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                        if tracked_box:
                            iou = self.compute_iou(tracked_box, [x, y, w, h])
                            iou_vis = min(iou / visible, 1.0)

            #                 cv2.putText(frame, f"IoU: {iou:.2f}", (x1, y2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # # Visualize
            # cv2.rectangle(frame, (int(l), int(t)), (int(r), int(b)), (0, 255, 0), 2)
            # cv2.putText(frame, f"ID {track_id}", (int(l), int(t) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # cv2.imshow("YOLOv8n + DeepSORT", frame)

        # At bottom of loop:
        elapsed = time.time() - start_time

        # fps = 1 / elapsed if elapsed > 0 else 0
        # print(f"Frame {frame_id}: {fps:.2f} FPS")

        msg_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        self.writer.writerow([msg_time, frame_id, self.desired_id, self.target_gt_id, elapsed, int(tracked_box[0]), int(tracked_box[1]), int(tracked_box[2]), int(tracked_box[3]), conf, iou, iou_vis, len(msg.data), img_name, float(freq), int(comp)])
        self.csvfile.flush()

    def __del__(self):
        # === Save results in MOT format ===
        if self.csvfile2:
            self.csvfile2.close()
        if self.csvfile:
            self.csvfile.close()

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