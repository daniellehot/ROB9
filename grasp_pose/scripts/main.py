#!/usr/bin/env python3

"""
Import sys modular, sys.argv The function of is to pass parameters from the outside to the inside of the program. sys.argv(number)，number=0 Is the name of the script
"""
import sys
import rospy
from affordancenet_service.srv import *
from realsense_service.srv import *
import numpy as np
import cv2

capture_new_scene = False

def captureNewScene():
    rospy.wait_for_service("/sensors/realsense/capture")
    captureService = rospy.ServiceProxy("/sensors/realsense/capture", capture)
    msg = capture()
    msg.data = True
    response = captureService(msg)
    print(response)

def getAffordanceResult():
    rospy.wait_for_service("/affordanceNet/result")
    affordanceNetService = rospy.ServiceProxy("/affordanceNet/result", affordance)
    msg = affordance()
    msg.data = True
    response = affordanceNetService(msg)

    no_objects = int(response.masks.layout.dim[0].size / 10)
    masks = np.asarray(response.masks.data).reshape((no_objects, int(response.masks.layout.dim[0].size / no_objects), response.masks.layout.dim[1].size, response.masks.layout.dim[2].size)) #* 255
    masks = masks.astype(np.uint8)

    bbox = np.asarray(response.bbox.data).reshape((-1,4))

    objects = np.asarray(response.object.data)
    print(masks.shape)
    print(masks.dtype)
    print(bbox)
    print(objects)
    for j in range(3):
        for i in range(10):
            print(j,i)
            cv2.imshow("title", masks[j][i])
            cv2.waitKey(0)

def main():

    # only capture a new scene at startup
    if capture_new_scene:
        captureNewScene()

    # takes a long time to run
    # make sure affordance-ros docker image is running
    getAffordanceResult()

if __name__ == "__main__":
    main()
