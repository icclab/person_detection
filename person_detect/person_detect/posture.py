#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time
from pose import getKeypoints, getValidPairs, getPersonwiseKeypoints
import blobconverter

NN_WIDTH, NN_HEIGHT = 456, 256

colors = [[0, 100, 255], [0, 100, 255], [0, 255, 255], [0, 100, 255], [0, 255, 255], [0, 100, 255], [0, 255, 0],
          [255, 200, 100], [255, 0, 255], [0, 255, 0], [255, 200, 100], [255, 0, 255], [0, 0, 255], [255, 0, 0],
          [200, 200, 0], [255, 0, 0], [200, 200, 0], [0, 0, 0]]
POSE_PAIRS = [[1, 2], [1, 5], [2, 3], [3, 4], [5, 6], [6, 7], [1, 8], [8, 9], [9, 10], [1, 11], [11, 12], [12, 13],
              [1, 0], [0, 14], [14, 16], [0, 15], [15, 17], [2, 17], [5, 16]]

pipeline = dai.Pipeline()

# Color camera
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setPreviewSize(NN_WIDTH, NN_HEIGHT)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setFps(30)

monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutSpatialData = pipeline.create(dai.node.XLinkOut)
xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

# Config
topLeft = dai.Point2f(0.0, 0.0)
bottomRight = dai.Point2f(1.0, 1.0)

config = dai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 100
config.depthThresholds.upperThreshold = 10000
calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
config.roi = dai.Rect(topLeft, bottomRight)

spatialLocationCalculator.inputConfig.setWaitForMessage(False)
spatialLocationCalculator.initialConfig.addROI(config)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
stereo.depth.link(spatialLocationCalculator.inputDepth)

spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

# Load NN blob
detection_nn = pipeline.create(dai.node.NeuralNetwork)
detection_nn.setBlobPath(blobconverter.from_zoo(name="human-pose-estimation-0001", shaves=6))

# Define manip
manip = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setResize(NN_WIDTH, NN_HEIGHT)
manip.initialConfig.setFrameType(dai.RawImgFrame.Type.BGR888p)
manip.inputConfig.setWaitForMessage(False)

# Create outputs
xout_cam = pipeline.create(dai.node.XLinkOut)
xout_cam.setStreamName("cam")

xout_nn = pipeline.create(dai.node.XLinkOut)
xout_nn.setStreamName("nn")

camRgb.preview.link(manip.inputImage)
camRgb.preview.link(xout_cam.input)
manip.out.link(detection_nn.input)
detection_nn.out.link(xout_nn.input)

def scale(point):
    return int(point[0] * scale_factor) + offset_w, int(point[1] * scale_factor)

# ----------------- Main Loop -----------------
with dai.Device(pipeline) as device:
    q_cam = device.getOutputQueue("cam", 4, blocking=False)
    q_nn = device.getOutputQueue("nn", 4, blocking=False)

    # Output queue will be used to get the depth frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
    spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig")

    start_time = time.time()
    counter = 0
    fps = 0

    keypoints_list = None
    detected_keypoints = None
    personwiseKeypoints = None

    while True:
        in_frame = q_cam.get()
        in_nn = q_nn.get()
        frame = in_frame.getCvFrame()

        # Get outputs
        heatmaps = np.array(in_nn.getLayerFp16('Mconv7_stage2_L2')).reshape((1, 19, 32, 57))
        pafs = np.array(in_nn.getLayerFp16('Mconv7_stage2_L1')).reshape((1, 38, 32, 57))
        heatmaps = heatmaps.astype('float32')
        pafs = pafs.astype('float32')

        outputs = np.concatenate((heatmaps, pafs), axis=1)

        new_keypoints = []
        new_keypoints_list = np.zeros((0, 3))
        keypoint_id = 0

        inDepth = depthQueue.get() # Blocking call, will wait until a new data has arrived

        depthFrame = inDepth.getFrame() # depthFrame values are in millimeters

        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0  # Set a default minimum depth value when all elements are zero
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)
        depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        for row in range(18):
            probMap = outputs[0, row, :, :]
            probMap = cv2.resize(probMap, (NN_WIDTH, NN_HEIGHT))
            keypoints = getKeypoints(probMap, 0.3)
            new_keypoints_list = np.vstack([new_keypoints_list, *keypoints])

            keypoints_with_id = []

            for i in range(len(keypoints)):
                keypoints_with_id.append(keypoints[i] + (keypoint_id,))
                keypoint_id += 1

            new_keypoints.append(keypoints_with_id)

        valid_pairs, invalid_pairs = getValidPairs(outputs, w=NN_WIDTH, h=NN_HEIGHT, detected_keypoints=new_keypoints)
        newPersonwiseKeypoints = getPersonwiseKeypoints(valid_pairs, invalid_pairs, new_keypoints_list)

        detected_keypoints, keypoints_list, personwiseKeypoints = (new_keypoints, new_keypoints_list, newPersonwiseKeypoints)

        if keypoints_list is not None and detected_keypoints is not None and personwiseKeypoints is not None:
            scale_factor = frame.shape[0] / NN_HEIGHT
            offset_w = int(frame.shape[1] - NN_WIDTH * scale_factor) // 2

            # right shoulder, left shoulder, right hip, right knee, right ankle, left hip, left knee
            my_det = [2, 5, 8, 9, 10, 11, 12]
   
            # right shoulder, left shoulder, right hip, left hip
            if len(detected_keypoints[my_det[0]]) > 0 and len(detected_keypoints[my_det[1]]) > 0 and len(detected_keypoints[my_det[2]]) > 0 and len(detected_keypoints[my_det[5]]) > 0:
                
                r_sho = scale(detected_keypoints[my_det[0]][0])
                l_sho = scale(detected_keypoints[my_det[1]][0])
                r_hip = scale(detected_keypoints[my_det[2]][0])
                l_hip = scale(detected_keypoints[my_det[5]][0])

                print("right shoulder =", r_sho)
                print("right hip =", r_hip)
                print("left shoulder =", l_sho)
                print("left hip =", l_hip)
                
                # right knee, left knee
                if len(detected_keypoints[my_det[3]]) > 0 and len(detected_keypoints[my_det[6]]) > 0:
                    
                    print("knees detected")

                    r_knee = scale(detected_keypoints[my_det[3]][0])
                    l_knee = scale(detected_keypoints[my_det[6]][0])

                    print("right knee =", r_knee)
                    print("left knee =", l_knee)

                    dy = r_knee[1] - r_sho[1]
                    dx = r_knee[0] - r_sho[0]
                    angle = np.degrees(np.arctan2(dy, dx))
                    print("angle from horizontal =", angle)

                    if np.abs(angle) >= 70.0 and np.abs(angle) <= 110.0:
                        print("Standing / sitting")

                    else:
                        print("lying down") 

                    # Normalize points to [0, 1] for ROI
                    x1 = min(r_sho[0], l_knee[0]) / frame.shape[1]
                    y1 = min(r_sho[1], l_knee[1]) / frame.shape[0]
                    x2 = max(r_sho[0], l_knee[0]) / frame.shape[1]
                    y2 = max(r_sho[1], l_knee[1]) / frame.shape[0]

                    config.roi = dai.Rect(dai.Point2f(x1, y1), dai.Point2f(x2, y2))
                    config.calculationAlgorithm = calculationAlgorithm
                    cfg = dai.SpatialLocationCalculatorConfig()
                    cfg.addROI(config)
                    spatialCalcConfigInQueue.send(cfg)
                
                    spatialData = spatialCalcQueue.get().getSpatialLocations()
                    
                    for depthData in spatialData:

                        print("x =", depthData.spatialCoordinates.x / 1000.0, "y =", -depthData.spatialCoordinates.y / 1000.0, "z =", depthData.spatialCoordinates.z / 1000.0)

                    cv2.circle(frame, scale(detected_keypoints[my_det[0]][0][0:2]), 5, colors[i], -1, cv2.LINE_AA)
                    cv2.circle(frame, scale(detected_keypoints[my_det[1]][0][0:2]), 5, colors[i], -1, cv2.LINE_AA)
                    cv2.circle(frame, scale(detected_keypoints[my_det[2]][0][0:2]), 5, colors[i], -1, cv2.LINE_AA)
                    cv2.circle(frame, scale(detected_keypoints[my_det[3]][0][0:2]), 5, colors[i], -1, cv2.LINE_AA)
                    cv2.circle(frame, scale(detected_keypoints[my_det[5]][0][0:2]), 5, colors[i], -1, cv2.LINE_AA)
                    cv2.circle(frame, scale(detected_keypoints[my_det[6]][0][0:2]), 5, colors[i], -1, cv2.LINE_AA)
                    cv2.line(frame, scale((r_sho[0], r_sho[1])), scale((r_hip[0], r_hip[1])), colors[i], 3, cv2.LINE_AA)
                    cv2.line(frame, scale((r_hip[0], r_hip[1])), scale((r_knee[0], r_knee[1])), colors[i], 3, cv2.LINE_AA)
                    cv2.line(frame, scale((l_sho[0], l_sho[1])), scale((l_hip[0], l_hip[1])), colors[i], 3, cv2.LINE_AA)
                    cv2.line(frame, scale((l_hip[0], l_hip[1])), scale((l_knee[0], l_knee[1])), colors[i], 3, cv2.LINE_AA)

                else: 
                    print("knees not detected")

                    dy = r_hip[1] - r_sho[1]
                    dx = r_hip[0] - r_sho[0]
                    angle = np.degrees(np.arctan2(dy, dx))
                    print("angle from horizontal  =", angle)
                   
                    if np.abs(angle) >= 70.0 and np.abs(angle) <= 110.0:
                        print("Standing / sitting")

                    else:
                        print("lying down") 

                    # Normalize points to [0, 1] for ROI
                    x1 = min(r_sho[0], l_hip[0]) / frame.shape[1]
                    y1 = min(r_sho[1], l_hip[1]) / frame.shape[0]
                    x2 = max(r_sho[0], l_hip[0]) / frame.shape[1]
                    y2 = max(r_sho[1], l_hip[1]) / frame.shape[0]

                    config.roi = dai.Rect(dai.Point2f(x1, y1), dai.Point2f(x2, y2))
                    config.calculationAlgorithm = calculationAlgorithm
                    cfg = dai.SpatialLocationCalculatorConfig()
                    cfg.addROI(config)
                    spatialCalcConfigInQueue.send(cfg)
                
                    spatialData = spatialCalcQueue.get().getSpatialLocations()
                    
                    for depthData in spatialData:

                        print("x =", depthData.spatialCoordinates.x / 1000.0, "y =", -depthData.spatialCoordinates.y / 1000.0, "z =", depthData.spatialCoordinates.z / 1000.0)

                    cv2.circle(frame, scale(detected_keypoints[my_det[0]][0][0:2]), 5, colors[i], -1, cv2.LINE_AA)
                    cv2.circle(frame, scale(detected_keypoints[my_det[1]][0][0:2]), 5, colors[i], -1, cv2.LINE_AA)
                    cv2.circle(frame, scale(detected_keypoints[my_det[2]][0][0:2]), 5, colors[i], -1, cv2.LINE_AA)
                    cv2.circle(frame, scale(detected_keypoints[my_det[5]][0][0:2]), 5, colors[i], -1, cv2.LINE_AA)
                    cv2.line(frame, scale((r_sho[0], r_sho[1])), scale((r_hip[0], r_hip[1])), colors[i], 3, cv2.LINE_AA)
                    cv2.line(frame, scale((l_sho[0], l_sho[1])), scale((l_hip[0], l_hip[1])), colors[i], 3, cv2.LINE_AA)

        # FPS
        counter += 1
        if (time.time() - start_time) > 1:
            fps = counter / (time.time() - start_time)
            counter = 0
            start_time = time.time()

        label_fps = "Fps: {:.2f}".format(fps)
        
        cv2.putText(frame, label_fps, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))

        # Show frame
        cv2.imshow("Pose Estimation", frame)
        if cv2.waitKey(1) == ord('q'):
            break
