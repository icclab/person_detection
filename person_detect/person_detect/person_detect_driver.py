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

class PersonDetectDriver(Node):

    def __init__(self):
        super().__init__('person_detect_driver')

        self.t = None

        self.time = rclpy.time.Time()

        self.from_frame_rel = 'oak_rgb_camera_optical_frame'
        self.to_frame_rel = 'map'
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, '/summit/person_marker', 10)

        self.timer1 = self.create_timer(0.5, self.get_pose)

        self.subscription = self.create_subscription(
            TrackDetection2DArray,
            '/summit/color/yolov4_Spatial_tracklets',
            self.tracking_callback,
            10
        )

    def get_pose(self):
        
        self.t = TransformStamped()
        
        try:
            self.t = self.tf_buffer.lookup_transform(
                self.to_frame_rel, self.from_frame_rel, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0)
            )


        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.from_frame_rel} to {self.to_frame_rel}: {ex}')
            return

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

                    p1 = PointStamped()
                    p1.header.frame_id = self.from_frame_rel
                    self.time = msg.header.stamp
                    p1.header.stamp = self.time
                    p1.point.x = hyp.pose.pose.position.x
                    p1.point.y = -hyp.pose.pose.position.y
                    p1.point.z = hyp.pose.pose.position.z  

                    p2 = tf2_geometry_msgs.do_transform_point(p1, self.t)

                    marker = Marker()
                    marker.header.frame_id = "map" 
                    marker.header.stamp = msg.header.stamp
                    marker.ns = "person"
                    marker.id = 0
                    marker.type = Marker.SPHERE 
                    marker.action = Marker.ADD

                    marker.pose.position.x = p2.point.x  
                    marker.pose.position.y = p2.point.y  
                    # marker.pose.position.z = p2.point.z 
                    marker.pose.position.z = 0.0

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
                    self.marker_pub.publish(marker)

                    # print("bbox: size: x", bbox_size_x, ", y: ", bbox_size_y)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()