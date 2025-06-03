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
        self.declare_parameter("img_folder", "/home/icc-nano/energy_ws/src/MOT20-01/img1/")
        self.declare_parameter("frequency", 1.0)  # default: 1 Hz
        self.declare_parameter("compress", 100) 
 
        self.img_folder = self.get_parameter("img_folder").get_parameter_value().string_value

        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value

        self.compress = self.get_parameter("compress").get_parameter_value().integer_value

        self.publisher = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.bridge = CvBridge()
 
        # === Count images ===
        self.image_files = sorted([
            f for f in os.listdir(self.img_folder)
            if f.endswith(".jpg") or f.endswith(".png")
        ])
        self.total_images = len(self.image_files)

        if self.total_images == 0:
            self.get_logger().error("No image files found.")
            rclpy.shutdown()
            return
 
        self.frame_id = 0
        period = 1.0 / self.frequency
        self.timer = self.create_timer(period, self.timer_callback)
 
        self.get_logger().info(
            f"Publishing {self.total_images} images at {self.frequency} Hz"
        )
 
    def timer_callback(self):
        if self.frame_id >= self.total_images:
            self.get_logger().info("All images published. Shutting down.")
            rclpy.shutdown()
            return
        
        filename = self.image_files[self.frame_id]
        img_path = os.path.join(self.img_folder, filename)
        frame = cv2.imread(img_path)

        if frame is None:
            self.get_logger().warn(f"Could not read image: {filename}")
            self.frame_id += 1
            return
 
        log_name = os.path.basename(img_path)
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = log_name + "," + str(self.compress) + "," + str(self.frequency)
        self.publisher.publish(ros_image)
 
        # self.get_logger().info(f'[{self.frequency:.1f} Hz] Published image: {log_name}')
        self.frame_id += 1
 
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 
 