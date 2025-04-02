#!/usr/bin/env python3

from pathlib import Path
import os
import sys
import cv2
import depthai as dai
import numpy as np
import time
import rclpy
from rclpy.node import Node
import math
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

class DetectPerson(Node):

    def __init__(self):
        super().__init__('person_detect')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        package_name = 'person_detect'
        blob = 'launch/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob'

        # Get argument first
        nnBlobPath = os.path.join(get_package_share_directory(package_name), blob)

        # Tiny yolo v3/4 label texts
        labelMap = ["person"]

        syncNN = True

        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        nnNetworkOut = pipeline.create(dai.node.XLinkOut)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutNN = pipeline.create(dai.node.XLinkOut)
        xoutDepth = pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")
        xoutDepth.setStreamName("depth")
        nnNetworkOut.setStreamName("nnNetwork")

        # Properties
        camRgb.setPreviewSize(416, 416)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")

        # setting node configs
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Align depth map to the perspective of RGB camera, on which inference is done
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
        stereo.setSubpixel(True)

        spatialDetectionNetwork.setBlobPath(nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        # Yolo specific parameters
        spatialDetectionNetwork.setNumClasses(80)
        spatialDetectionNetwork.setCoordinateSize(4)
        spatialDetectionNetwork.setAnchors([10,14, 23,27, 37,58, 81,82, 135,169, 344,319])
        spatialDetectionNetwork.setAnchorMasks({ "side26": [1,2,3], "side13": [3,4,5] })
        spatialDetectionNetwork.setIouThreshold(0.5)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb.preview.link(spatialDetectionNetwork.input)
        if syncNN:
            spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)

        spatialDetectionNetwork.out.link(xoutNN.input)

        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
        spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)

        self.device = dai.Device(pipeline) 

        self.printOutputLayersOnce = True

        self.t = None

        self.from_frame_rel = 'oak_rgb_camera_optical_frame'
        self.to_frame_rel = 'map'

        self.timer1 = self.create_timer(0.5, self.get_pose)

        self.timer2 = self.create_timer(1.0, self.detect_person)

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

    def detect_person(self):
        '''
        Spatial Tiny-yolo example
        Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
        Can be used for tiny-yolo-v3 or tiny-yolo-v4 networks
        '''
        # color = (255, 255, 255)

        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        previewQueue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        detectionNNQueue = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        networkQueue = self.device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False)

        inPreview = previewQueue.get()
        inDet = detectionNNQueue.get()
        depth = depthQueue.get()
        inNN = networkQueue.get()

        if self.printOutputLayersOnce:
            toPrint = 'Output layer names:'
            for ten in inNN.getAllLayerNames():
                toPrint = f'{toPrint} {ten},'
            print(toPrint)
            self.printOutputLayersOnce = False

        frame = inPreview.getCvFrame()
        depthFrame = depth.getFrame() # depthFrame values are in millimeters

        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0  # Set a default minimum depth value when all elements are zero
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)
        depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        detections = inDet.detections

        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]

        for detection in detections:

            if detection.label == 0:
                roiData = detection.boundingBoxMapping
                roi = roiData.roi
                roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                topLeft = roi.topLeft()
                bottomRight = roi.bottomRight()
                xmin = int(topLeft.x)
                ymin = int(topLeft.y)
                xmax = int(bottomRight.x)
                ymax = int(bottomRight.y)
                # cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, 1)

                # Denormalize bounding box
                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)

                # print("x =", int(detection.spatialCoordinates.x), "y =", -int(detection.spatialCoordinates.y), "z =", int(detection.spatialCoordinates.z))
                # print("x =", detection.spatialCoordinates.x, "y =", -detection.spatialCoordinates.y, "z =", detection.spatialCoordinates.z)

                # t0 = TransformStamped()

                time_now = self.get_clock().now()

                # # Read message content and assign it to
                # # corresponding tf variables
                # t0.header.stamp = time_now.to_msg()
                # t0.header.frame_id = self.from_frame_rel
                # t0.child_frame_id = 'person'

                # # Turtle only exists in 2D, thus we get x and y translation
                # # coordinates from the message and set the z coordinate to 0
                # t0.transform.translation.x = detection.spatialCoordinates.x / 1000.0
                # t0.transform.translation.y = -detection.spatialCoordinates.y / 1000.0
                # t0.transform.translation.z = detection.spatialCoordinates.z / 1000.0

                # # Send the transformation
                # self.tf_broadcaster.sendTransform(t0)
                
                p1 = PointStamped()
                p1.header.frame_id = self.from_frame_rel
                p1.header.stamp = time_now.to_msg()
                p1.point.x = detection.spatialCoordinates.x / 1000.0
                p1.point.y = -detection.spatialCoordinates.y / 1000.0
                p1.point.z = detection.spatialCoordinates.z / 1000.0

                p2 = tf2_geometry_msgs.do_transform_point(p1, self.t)

                marker = Marker()
                marker.header.frame_id = "map" 
                marker.header.stamp = time_now.to_msg()
                marker.ns = "person"
                marker.id = 0
                marker.type = Marker.SPHERE 
                marker.action = Marker.ADD

                marker.pose.position.x = p2.point.x  
                marker.pose.position.y = p2.point.y  
                marker.pose.position.z = p2.point.z 

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0

                self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    detectperson = DetectPerson()

    executor = MultiThreadedExecutor()
    executor.add_node(detectperson)

    executor.spin()


if __name__ == '__main__':
    main()