#!/usr/bin/env python3

"""
Import sys modular, sys.argv The function of is to pass parameters from the outside to the inside of the program. sys.argv(number)，number=0 Is the name of the script
"""
import sys
import rospy
from affordancenet_service.srv import *
import numpy as np
import cv2

def getAffordanceResult():
    rospy.wait_for_service("/affordanceNet/result")
    affordanceNetService = rospy.ServiceProxy("/affordanceNet/result", affordance)
    msg = affordance()
    msg.data = True
    response = affordanceNetService(msg)

    no_objects = int(response.masks.layout.dim[0].size / 10)
    masks = np.asarray(response.masks.data).reshape((no_objects, response.masks.layout.dim[0].size, response.masks.layout.dim[1].size, response.masks.layout.dim[2].size))

    bbox = np.asarray(response.bbox.data).reshape((-1,4))

    objects = np.asarray(response.object.data)

if __name__ == "__main__":
    print("Usage example of client server service")

    print("Telling affordanceNET to analyze image from realsense and return predictions.")
    getAffordanceResult()
