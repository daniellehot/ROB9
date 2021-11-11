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
from cameraClient import CameraClient

if __name__ == "__main__":

    cam = CameraClient(type = "realsenseD435")

    cam.captureNewScene()

    cam.getRGB()
    cv2.imshow("rgb image", cam.rgb)
    cv2.waitKey(0)

    cam.getDepth()
    cv2.imshow("depth image", (cam.depth*255).astype(np.uint8))
    cv2.waitKey(0)

    cam.getUvStatic()
    print(cam.uv.shape)
    print(cam.uv[0:10])
