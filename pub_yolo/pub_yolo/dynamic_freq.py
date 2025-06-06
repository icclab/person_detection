import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import csv
import math
import subprocess
import signal
from rclpy.executors import MultiThreadedExecutor

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter("image_folder", "/home/icc-nano/energy_ws/src/MOT20-01/img1/")
        self.declare_parameter("csv_file", "/home/icc-nano/energy_ws/src/workload_1.csv")

        self.declare_parameter("img_start_index", 0)

        self.declare_parameter("compress", 100) 
        self.declare_parameter("fps", 30.0)  # default: 30 Hz

        self.image_folder = self.get_parameter("image_folder").get_parameter_value().string_value

        self.fps = self.get_parameter("fps").get_parameter_value().double_value

        self.compress = self.get_parameter("compress").get_parameter_value().integer_value

        self.img_start_index = self.get_parameter("img_start_index").get_parameter_value().integer_value

        self.publisher = self.create_publisher(Image, '/oak/rgb/image_raw', 10)

        self.csv_file = self.get_parameter("csv_file").get_parameter_value().string_value
 
        self.bridge = CvBridge()

        # === Count images ===
        self.image_files = sorted([
            f for f in os.listdir(self.image_folder)
            if f.endswith(".jpg") or f.endswith(".png")
        ])

        if len(self.image_files) < 1:
            raise ValueError("No images.")
        self.total_images = len(self.image_files)

        # Read frequency list
        self.frequencies = []
        with open(self.csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                freq = float(row['frequency(Hz)']) 
                if freq <= 0.0:
                    freq = 1.0  # default minimum frequency
                self.frequencies.append(freq)

        self.current_freq_index = 0
        self.image_index = self.img_start_index
        self.add = 0 
        self.images_published_in_current_block = 0

        self.start_frequency_block()

    def start_frequency_block(self):
        if self.current_freq_index >= len(self.frequencies):
            self.get_logger().warn('Finished all frequency blocks.')
            rclpy.shutdown()
            return

        self.current_freq = self.frequencies[self.current_freq_index]
        self.delay = 1.0 / self.current_freq
        self.images_to_publish = self.current_freq  # since 1 second block
        self.images_published_in_current_block = 0

        self.get_logger().info(
            f"Frequency {self.current_freq} Hz: publishing {self.images_to_publish} images in 1 second"
        )

        # Start the timer at current frequency
        self.timer = self.create_timer(self.delay, self.timer_callback)

    def timer_callback(self):
        if self.images_published_in_current_block >= self.images_to_publish:
            self.timer.cancel()
            self.current_freq_index += 1
            self.image_index -= round(self.fps / self.current_freq)
            self.image_index += 1
            self.start_frequency_block()
            return

        filename = self.image_files[self.image_index]
        img_path = os.path.join(self.image_folder, filename)

        cv_image = cv2.imread(img_path)
        if cv_image is None:
            self.get_logger().warning(f'Failed to load image: {img_path}')
            return

        log_name = os.path.basename(img_path)

        self.get_logger().info(
            f'[{self.current_freq} Hz] Published image {self.image_index + 1}: {log_name}'
        )

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = log_name + "," + str(self.compress) + "," + str(self.current_freq)
        self.publisher.publish(ros_image)
        
        self.image_index += round(self.fps / self.current_freq)
        self.images_published_in_current_block += 1

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except Exception as e:
        print(f"[ImagePublisher Error] {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
