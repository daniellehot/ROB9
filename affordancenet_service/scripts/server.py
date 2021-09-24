#!/usr/bin/env python3
from __future__ import print_function

from affordancenet_service.srv import affordance, affordanceResponse
from std_msgs.msg import MultiArrayDimension
import rospy
import numpy as np

def sendAffordanceResults(command):
    intToLabel = {0: 'class', 1: 'height', 2: 'width'}
    msg = affordanceResponse()

    # Example data only
    masks = np.ones((10,244,244))
    bbox = np.array([[1, 2, 3, 4], [2, 4, 6, 8]])
    detections = np.array([1, 3])

    # constructing mask message
    for i in range(3):
        dimMsg = MultiArrayDimension()
        dimMsg.label = intToLabel[i]
        stride = 1
        for j in range(3-i):
            stride = stride * masks.shape[i+j]
        dimMsg.stride = stride
        dimMsg.size = masks.shape[i]
        msg.masks.layout.dim.append(dimMsg)
    masks = masks.flatten().astype(int).tolist()
    msg.masks.data = masks

    # constructing bounding box message
    msg.bbox.data = bbox.flatten().astype(int).tolist()

    # constructing object detection class message
    msg.object.data = detections.flatten().tolist()

    return msg


    #return msg

if __name__ == "__main__":
    rospy.init_node('affordanceNet_service')
    serviceCapture = rospy.Service('/affordanceNet/result', affordance, sendAffordanceResults)
    rospy.spin()
