#!/usr/bin/env python
from __future__ import print_function

from realsense_service.srv import intrinsics, intrinsicsResponse
from realsense_service.srv import capture, captureResponse
from realsense_service.srv import depth, depthResponse
from realsense_service.srv import rgb, rgbResponse
import rospy
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge

def sendIntrinsics():
    pass

def sendDepth(command):
    global depth_image
    msg = depthResponse()
    msg.img.data = depth_image.flatten()
    msg.width.data = depth_image.shape[1]
    msg.height.data = depth_image.shape[0]
    return msg

def sendRGB(command):
    global color_image
    br = CvBridge()
    return br.cv2_to_imgmsg(color_image)



def captureRealsense(capture):
    global depth_image, color_image, captured
    try:
        align_to = rs.stream.color
        align = rs.align(align_to)

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        captured = True
        msg = captureResponse()
        msg.success.data = True
        return msg
    except:
        capture = False
        msg = captureResponse()
        msg.success.data = False
        return msg

if __name__ == "__main__":
    global profile, pipeline, captured, color_image, depth_image

    # Initially no images are available
    captured = False

    cam_width = 1280
    cam_height = 720

    pipeline = rs.pipeline()
    config = rs.config()

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.color, cam_width, cam_height, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, cam_width, cam_height, rs.format.z16, 30)
    profile = pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    captured = True

    rospy.init_node('realsense_service')
    serviceCapture = rospy.Service('/sensors/realsense/capture', capture, captureRealsense)
    serviceCapture = rospy.Service('/sensors/realsense/depth', depth, sendDepth)
    serviceCapture = rospy.Service('/sensors/realsense/rgb', rgb, sendRGB)
    rospy.spin()
