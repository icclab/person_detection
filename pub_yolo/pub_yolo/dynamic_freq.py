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

class ImagePublisher(Node):
    def __init__(self, image_folder, csv_file):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.bridge = CvBridge()

        self.image_files = sorted([
            os.path.join(image_folder, f)
            for f in os.listdir(image_folder)
            if f.endswith(('.jpg', '.png'))
        ])

        if len(self.image_files) < 1:
            raise ValueError("No images.")
        self.total_images = len(self.image_files)

        # Read frequency list
        self.frequencies = []
        self.compressions = []
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                freq = math.ceil(float(row['frequency(Hz)']))  # round down
                if freq <= 0:
                    freq = 1  # default minimum frequency
                self.frequencies.append(freq)
                self.compressions.append(float(row['compression_ratio(%)']))

        self.current_freq_index = 0
        self.image_index = 0
        self.images_published_in_current_block = 0

        self.current_compression = 100

        self.start_time_str = time.strftime("%d-%m-%Y_%H-%M-%S")
        self.output_file = f"imgs_tiny_yolo_v4_{self.start_time_str}.csv"

        self.csvfile = open(self.output_file, "w", newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["img_name", "bool_person"])
        self.csvfile.flush()

        self.proc = None

        # Start publishing the first frequency block
        self.start_frequency_block()

    def launch_compression_node(self, compress_quality: int):
        # Prepare the command with the compression quality argument
        # command = [
        #     'ros2', 'launch', 'person_detect', 'compress.launch.py',  f'compress:={compress_quality}'  # Pass the compression parameter
        # ]

        self.proc = subprocess.Popen([
            'ros2', 'run', 'image_transport', 'republish',
            'raw', 'compressed',
            '--ros-args',
            '-r', 'in:=/oak/rgb/image_raw',
            '-r', 'out/compressed:=/oak/rgb/image_raw/dynamic/compressed',
            '-p', f'out.jpeg_quality:={compress_quality}'
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, preexec_fn=os.setsid)

        self.get_logger().info(f"Launching compression node with JPEG quality {compress_quality}")
        
    def stop_compression_node(self):    
        if hasattr(self, 'proc') and self.proc and self.proc.poll() is None:
            self.get_logger().info("Terminating compression node...")
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)

        time.sleep(10)
        # time.sleep(7.5)

    def start_frequency_block(self):
        if self.current_freq_index >= len(self.frequencies):
            self.get_logger().info('Finished all frequency blocks.')
            rclpy.shutdown()
            return

        self.current_freq = self.frequencies[self.current_freq_index]
        self.delay = 1.0 / self.current_freq
        self.images_to_publish = self.current_freq  # since 1 second block
        self.images_published_in_current_block = 0

        self.get_logger().info(
            f"Frequency {self.current_freq} Hz: publishing {self.images_to_publish} images in 1 second"
        )

        self.current_compression = self.compressions[self.current_freq_index]
        self.launch_compression_node(int(self.current_compression))     

        # Start the timer at current frequency
        self.timer = self.create_timer(self.delay, self.timer_callback)

    def timer_callback(self):
        if self.images_published_in_current_block >= self.images_to_publish:
            self.timer.cancel()
            self.stop_compression_node()
            self.current_freq_index += 1
            self.start_frequency_block()
            return

        # Loop image index if we reach the end
        img_path = self.image_files[self.image_index % self.total_images]
        self.image_index += 1

        cv_image = cv2.imread(img_path)
        if cv_image is None:
            self.get_logger().warning(f'Failed to load image: {img_path}')
            return

        self.get_logger().info(
            f'[{self.current_freq} Hz] Published image {self.image_index + 1}/10: {os.path.basename(img_path)[0]}'
        )

        log_name = os.path.basename(img_path)
        self.writer.writerow([log_name, log_name[0]])
        self.csvfile.flush()

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = log_name + "," + str(int(self.current_compression))
        self.publisher.publish(ros_image)
        
        self.images_published_in_current_block += 1


def main(args=None):
    rclpy.init(args=args)
    image_folder = '/home/icc-nano/energy_ws/src/test_person'
    csv_file = '/home/icc-nano/energy_ws/src/workload.csv'
    node = ImagePublisher(image_folder, csv_file)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
