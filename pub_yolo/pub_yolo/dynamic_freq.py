import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import csv
import math

class ImagePublisher(Node):
    def __init__(self, image_folder, csv_file):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Load 10 images
        self.image_files = sorted([
            os.path.join(image_folder, f)
            for f in os.listdir(image_folder)
            if f.endswith(('.jpg', '.png'))
        ])[:10]  # limit to 10 images

        if len(self.image_files) < 10:
            raise ValueError("You must have at least 10 images.")

        # Read frequency list
        self.frequencies = []
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                freq = math.floor(float(row['frequency(Hz)']))  # round down
                if freq <= 0:
                    freq = 1  # default minimum frequency
                self.frequencies.append(freq)

        self.current_freq_index = 0
        self.image_index = 0

        # Start publishing the first frequency block
        self.start_frequency_block()

    def start_frequency_block(self):
        if self.current_freq_index >= len(self.frequencies):
            self.get_logger().info('Finished all frequency blocks.')
            rclpy.shutdown()
            return

        self.current_freq = self.frequencies[self.current_freq_index]
        self.delay = 1.0 / self.current_freq
        self.image_index = 0

        self.get_logger().info(
            f"Starting frequency block: {self.current_freq} Hz "
            f"(delay = {self.delay:.2f}s)"
        )

        # Start publishing 10 images at current frequency
        self.timer = self.create_timer(self.delay, self.timer_callback)

    def timer_callback(self):
        if self.image_index >= len(self.image_files):
            self.timer.cancel()
            self.current_freq_index += 1
            self.start_frequency_block()
            return

        img_path = self.image_files[self.image_index]
        cv_image = cv2.imread(img_path)
        if cv_image is None:
            self.get_logger().warning(f'Failed to load image: {img_path}')
            self.image_index += 1
            return

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(ros_image)

        self.get_logger().info(
            f'[{self.current_freq} Hz] Published image {self.image_index + 1}/10: {os.path.basename(img_path)}'
        )
        self.image_index += 1


def main(args=None):
    rclpy.init(args=args)
    image_folder = '/home/lei/Downloads/energy_ws/voc2005_2/VOC2005_2/PNGImages/INRIA_inria-person-test/'
    csv_file = '/home/lei/Downloads/energy_ws/image_frequencies.csv'
    node = ImagePublisher(image_folder, csv_file)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
