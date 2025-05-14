import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImagePublisher(Node):
    def __init__(self, image_folder):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.bridge = CvBridge()

        self.image_files = sorted([
            os.path.join(image_folder, f)
            for f in os.listdir(image_folder)
            if f.endswith(('.jpg', '.png'))
        ])
        self.index = 0

    def timer_callback(self):
        if self.index >= len(self.image_files):
            self.get_logger().info('Finished publishing all images.')
            rclpy.shutdown()
            return

        img_path = self.image_files[self.index]
        cv_image = cv2.imread(img_path)
        if cv_image is None:
            self.get_logger().warning(f'Failed to load image: {img_path}')
            self.index += 1
            return

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(ros_image)

        self.get_logger().info(f'Published image: {img_path}')
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    image_folder = '/home/icc-nano/energy_ws/src/person_detection/test_images/'
    node = ImagePublisher(image_folder)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()