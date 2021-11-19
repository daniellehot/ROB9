#!/usr/bin/env python3
import rospy
from affordance_analyzer.srv import getAffordanceSrv, getAffordanceSrvResponse
import numpy as np
import cv2
from cameraService.cameraClient import CameraClient
from affordanceService.client import AffordanceClient

if __name__ == "__main__":
    print("Usage example of client server service")
    rospy.init_node('affordance_analyzer_client', anonymous=True)

    affClient = AffordanceClient()
    affClient.start(GPU=True) # GPU acceleratio True or False

    print("Telling affordanceNET to analyze image from realsense and return predictions.")
    success = affClient.run(CONF_THRESHOLD = 0.3)
    _, _ = affClient.getAffordanceResult()

    success = affClient.processMasks(conf_threshold = 50, erode_kernel=(21,21))
    success = affClient.visualizeMasks()
    success = affClient.visualizeBBox()


    print("Found the following objects")
    for i in range(len(affClient.objects)):
        print(affClient.OBJ_CLASSES[affClient.objects[i]])
