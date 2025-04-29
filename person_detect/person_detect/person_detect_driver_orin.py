#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from depthai_ros_msgs.msg import TrackDetection2DArray
from geometry_msgs.msg import TransformStamped, PointStamped
from rclpy.executors import ExternalShutdownException
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker

from builtin_interfaces.msg import Duration

class PersonDetectDriverOrin(Node):

    def __init__(self):
        super().__init__('person_detect_driver')

        self.j = 0

        self.marker_pub = self.create_publisher(Marker, '/orin/person_marker', 10)

        self.subscription = self.create_subscription(
            TrackDetection2DArray,
            '/orin/color/yolov4_Spatial_tracklets',
            self.tracking_callback,
            10
        )

    def tracking_callback(self, msg):

        for i, detection in enumerate(msg.detections):
            bbox = detection.bbox
            
            # in mm
            bbox_center_x = bbox.center.position.x
            bbox_center_y = bbox.center.position.y
            bbox_size_x = bbox.size_x
            bbox_size_y = bbox.size_y

            results = detection.results
            is_tracking = detection.is_tracking
            tracking_id = detection.tracking_id
            tracking_age = detection.tracking_age
            tracking_status = detection.tracking_status

            for hyp in results:
                if int(hyp.hypothesis.class_id) == 0:

                    marker = Marker()
                    marker.header.frame_id = "oak_rgb_camera_optical_frame" 
                    marker.header.stamp = msg.header.stamp
                    marker.ns = "person"
                    marker.id = self.j
                    marker.type = Marker.SPHERE 
                    marker.action = Marker.ADD

                    marker.pose.position.x = hyp.pose.pose.position.x
                    marker.pose.position.y = -hyp.pose.pose.position.y
                    marker.pose.position.z = hyp.pose.pose.position.z  

                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
                    marker.scale.z = 0.2

                    if bbox_size_x <= bbox_size_y:
                        self.get_logger().info("Person is standing")
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        
                    else:
                        self.get_logger().info("Person is lying down")
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0

                    marker.color.a = 1.0
                    marker.lifetime = Duration(sec=10*60, nanosec=0)  # 10 minutes
                    self.marker_pub.publish(marker)

                    # print("bbox: size: x", bbox_size_x, ", y: ", bbox_size_y)

                    self.j += 1


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectDriverOrin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()