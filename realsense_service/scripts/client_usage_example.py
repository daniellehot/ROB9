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
    img = np.frombuffer(response.img.data, dtype=np.uint8).reshape(response.img.height, response.img.width, -1)
    cv2.imshow("RGB", img)
    cv2.waitKey(0)


def getDepth():
    rospy.wait_for_service("/sensors/realsense/depth")
    depthService = rospy.ServiceProxy("/sensors/realsense/depth", depth)
    msg = depth()
    msg.data = True
    response = depthService(msg)
    br = CvBridge()
    img = br.imgmsg_to_cv2(response.img, desired_encoding='passthrough')
    img = img * 255
    cv2.imshow("Depth", img)
    cv2.waitKey(0)

def getUvStatic():
    rospy.wait_for_service("/sensors/realsense/pointcloudGeometry/static/uv")
    uvStaticService = rospy.ServiceProxy("/sensors/realsense/pointcloudGeometry/static/uv", uvSrv)
    msg = uvSrv()

    msg.data = True
    response = uvStaticService(msg)
    rows = int(response.uv.layout.dim[0].size / response.uv.layout.dim[1].size)
    cols = int(response.uv.layout.dim[1].size)

    uvStatic = np.array(response.uv.data).astype(int)
    uvStatic = np.reshape(uvStatic, (rows, cols))

    # y value stored in first indice, x stored in second
    for uv in uvStatic:
        print(uv)

if __name__ == "__main__":
    print("Usage example of client server service")

    print("Telling camera to capture depth and rgb images")
    captureRealsense()

    print("Telling camera to send UV indices")
    getUvStatic()

    print("Telling camera to send captured depth image")
    getDepth()

    print("Telling camera to send captured rgb image")
    getRGB()
