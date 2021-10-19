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
    print("Captured scene: ", response.success.data)
    return response.success.data

def getRGB():
    rospy.wait_for_service("/sensors/realsense/rgb")
    rgbService = rospy.ServiceProxy("/sensors/realsense/rgb", rgb)
    msg = rgb()
    msg.data = True
    response = rgbService(msg)
    br = CvBridge()
    img = br.imgmsg_to_cv2(response.img)
    return img


def getDepth():
    rospy.wait_for_service("/sensors/realsense/depth")
    depthService = rospy.ServiceProxy("/sensors/realsense/depth", depth)
    msg = depth()
    msg.data = True
    response = depthService(msg)
    w = response.width.data
    h = response.height.data
    depthImg = np.asarray(response.img.data).reshape((h,w)) # use this depth image
    #vis0 = np.asarray(response.img.data).reshape((h,w)) / 256.0
    #vis0 = vis0.astype(np.ubyte)
    #vis0 = np.uint8(255-vis0) # only for visualization purposes
    #cv2.imshow("Depth image visualization", vis0)
    #cv2.waitKey(0)
    return depthImg
