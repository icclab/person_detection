import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, time
import os
import csv
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
        
        self.start_time_str = time.strftime("%d-%m-%Y_%H-%M-%S")
        self.output_file = f"tiny_yolo_v4_{self.start_time_str}.csv"

        self.csvfile = open(self.output_file, "w", newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["unix_timestamp_sec", "class_id", "inference_time_sec", "accuracy_in_percent", "payload_bytes", "cuda", "person_bool"])
        self.csvfile.flush()

        self.get_logger().info(f"Logging to: {self.output_file}")

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

        start = time.time()
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        end = time.time()

        class_name = "None"
        conf = 0
        max_conf = 0
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

        self.get_logger().info(f"Detected {class_name} with max. confidence {max_conf:.2f}")

        if max_conf > 0 and class_name == "person":
            self.person_bool = 1
        else:
            self.person_bool = 0

        self.writer.writerow([end, class_name, end - start, max_conf * 100, len(msg.data), self.use_cuda, self.person_bool])
        self.csvfile.flush()

    def __del__(self):
        if self.csvfile:
            self.csvfile.close()

def main(args=None):
    rclpy.init(args=args)
    node = YoloV4TinyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()