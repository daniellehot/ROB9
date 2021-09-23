#!/usr/bin/env python3

"""
Import sys modular, sys.argv The function of is to pass parameters from the outside to the inside of the program. sys.argv(number)，number=0 Is the name of the script
"""
import sys
import rospy
from realsense_service.srv import *
import numpy as np
import cv2
from cv_bridge import CvBridge

def captureRealsense():
    rospy.wait_for_service("/sensors/realsense/capture")
    captureService = rospy.ServiceProxy("/sensors/realsense/capture", capture)
    msg = capture()
    msg.data = True
    response = captureService(msg)
    print(response)

def getRGB():
    rospy.wait_for_service("/sensors/realsense/rgb")
    rgbService = rospy.ServiceProxy("/sensors/realsense/rgb", rgb)
    msg = rgb()
    msg.data = True
    response = rgbService(msg)
    br = CvBridge()
    img = br.imgmsg_to_cv2(response.img)
    cv2.imshow("RGB", img)
    cv2.waitKey(0)


def getDepth():
    rospy.wait_for_service("/sensors/realsense/depth")
    depthService = rospy.ServiceProxy("/sensors/realsense/depth", depth)
    msg = depth()
    msg.data = True
    response = depthService(msg)
    w = response.width.data
    h = response.height.data
    depthImg = np.asarray(response.img.data).reshape((h,w)) # use this depth image
    vis0 = np.asarray(response.img.data).reshape((h,w)) / 256.0
    vis0 = vis0.astype(np.ubyte)
    vis0 = np.uint8(255-vis0) # only for visualization purposes
    cv2.imshow("Depth image visualization", vis0)
    cv2.waitKey(0)

if __name__ == "__main__":
    print("Usage example of client server service")

    print("Telling camera to capture depth and rgb images")
    captureRealsense()

    print("Telling camera to send captured depth image")
    getDepth()

    print("Telling camera to send captured rgb image")
    getRGB()
