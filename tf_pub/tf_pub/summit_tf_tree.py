import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener

class SummitTFListener(Node):
    def __init__(self):
        super().__init__('summit_tf_listener')

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pose publisher
        self.pose_publisher = self.create_publisher(PoseStamped, '/summit/base_pose', 10)

        # Timer to check TF data periodically
        self.timer = self.create_timer(1, self.update_pose)

    def update_pose(self):
        """Fetch the transform from /map to /base_link and publish the pose."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0)
            )

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation

            # Publish pose
            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Published Pose: x={pose_msg.pose.position.x}, y={pose_msg.pose.position.y}, z={pose_msg.pose.position.z}")

        except tf2_ros.LookupException:
            self.get_logger().warn("Transform from 'map' to 'base_footprint' not found!")

def main(args=None):
    rclpy.init(args=args)
    node = SummitTFListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


