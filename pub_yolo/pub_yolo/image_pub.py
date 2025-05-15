import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
 
class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
 
        # Declare and get parameters
        self.declare_parameter("image_folder", "/home/icc-nano/energy_ws/src/test_person/")
        self.declare_parameter("frequency", 1.0)  # default: 1 Hz
        self.declare_parameter("duration", 200.0)  # default: 200 seconds
 
        self.image_folder = self.get_parameter("image_folder").get_parameter_value().string_value
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.duration = self.get_parameter("duration").get_parameter_value().double_value
 
        self.publisher = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.bridge = CvBridge()
 
        self.image_files = sorted([
            os.path.join(self.image_folder, f)
            for f in os.listdir(self.image_folder)
            if f.lower().endswith(('.jpg', '.png'))
        ])
        self.total_images = len(self.image_files)
 
        if self.total_images == 0:
            self.get_logger().error("No image files found.")
            rclpy.shutdown()
            return
 
        self.index = 0
        self.start_time = self.get_clock().now()
 
        period = 1.0 / self.frequency
        self.timer = self.create_timer(period, self.timer_callback)
 
        self.get_logger().info(
            f"Publishing {self.total_images} images in a loop at {self.frequency} Hz for {self.duration} seconds"
        )
 
    def timer_callback(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time >= self.duration:
            self.get_logger().info("Finished 200 seconds of publishing.")
            rclpy.shutdown()
            return
 
        img_path = self.image_files[self.index % self.total_images]
        cv_image = cv2.imread(img_path)
 
        if cv_image is None:
            self.get_logger().warning(f'Failed to load image: {img_path}')
            self.index += 1
            return
 
        log_name = os.path.basename(img_path)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = log_name[0] + "," + log_name + "," + str(100) + "," + str(self.frequency)
        self.publisher.publish(ros_image)
 
        self.get_logger().info(f'[{self.frequency:.1f} Hz] Published image: {log_name}')
        self.index += 1
 
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 
 