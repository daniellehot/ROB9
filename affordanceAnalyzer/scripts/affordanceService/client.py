#!/usr/bin/env python3

import sys
import time
import copy
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cameraService.cameraClient import CameraClient
from affordance_analyzer.srv import *

class AffordanceClient(object):
    """docstring for AffordanceClient."""

    def __init__(self):

        self.no_objects = 0
        self.masks = None
        self.bbox = None
        self.objects = None
        self.scores = None
        self.GPU = False
        self.name = self.getName()

        if self.name == "affordancenet":
            self.noObjClass = 11
            self.noLabelClass = 10
            self.OBJ_CLASSES = ('__background__', 'bowl', 'tvm', 'pan', 'hammer', 'knife',
                                    'cup', 'drill', 'racket', 'spatula', 'bottle')
            self.OBJ_NAME_TO_ID = {'__background__': 0, 'bowl': 1, 'tvm': 2, 'pan': 3,
                                'hammer': 4, 'knife': 5, 'cup': 6, 'drill': 7,
                                'racket': 8, 'spatula': 9, 'bottle': 10}
            self.labelsToNames = {'__background__': 0, 'contain': 1, 'cut': 2, 'display': 3, 'engine': 4, 'grasp': 5, 'hit': 6, 'pound': 7, 'support': 8, 'w-grasp': 9}
            self.graspLabels = [5, 9]
            self.functionalLabels = [1, 2, 3, 4, 6, 7, 8]

        elif self.name == "affordancenet_context":
            self.noObjClass = 2
            self.noLabelClass = 8
            self.OBJ_CLASSES = ('__background__', 'objectness')
            self.OBJ_NAME_TO_ID = {'__background__': 0, 'objectness': 1}
            self.labelsToNames = {'__background__': 0, 'grasp': 1, 'cut': 2, 'scoop': 3, 'contain': 4, 'pound': 5, 'support': 6, 'w-grasp': 7}
            self.graspLabels = [1, 7]
            self.functionalLabels = [2, 3, 4, 5, 6]

    def getName(self):

        rospy.wait_for_service("/affordance/name")
        nameService = rospy.ServiceProxy("/affordance/name", getNameSrv)
        msg = getNameSrv()
        msg.data = True
        response = nameService(msg)

        return response.name.data


    def start(self, GPU=False):
        self.GPU = GPU

        rospy.wait_for_service("/affordance/start")
        startAffordanceNetService = rospy.ServiceProxy("/affordance/start", startAffordanceSrv)
        msg = startAffordanceSrv()
        msg.data = GPU
        response = startAffordanceNetService(msg)

        return response.status.data


    def stop(self):
        rospy.wait_for_service("/affordance/stop")
        stopAffordanceNetService = rospy.ServiceProxy("/affordance/stop", stopAffordanceSrv)
        msg = stopAffordanceSrv()
        msg.data = True
        response = startAffordanceNetService(msg)

        return response.status.data

    def run(self, img, CONF_THRESHOLD = 0.7):
        rospy.wait_for_service("/affordance/run")
        runAffordanceNetService = rospy.ServiceProxy("/affordance/run", runAffordanceSrv)

        imgMsg = Image()
        imgMsg.height = img.shape[0]
        imgMsg.width = img.shape[1]
        imgMsg.data = img.flatten().tolist()

        response = runAffordanceNetService(imgMsg, Float32(CONF_THRESHOLD))

        return response.success.data

    def getAffordanceResult(self):

        s1 = time.time()
        rospy.wait_for_service("/affordance/result")
        affordanceNetService = rospy.ServiceProxy("/affordance/result", getAffordanceSrv)
        #print("Waiting for affordance service took: ", (time.time() - s1) * 1000 )

        s2 = time.time()
        msg = getAffordanceSrv()
        msg.data = True
        response = affordanceNetService(msg)
        #print("Receiving data from affordance service took: ", (time.time() - s2) * 1000 )

        s3 = time.time()
        no_objects = int(response.masks.layout.dim[0].size / self.noLabelClass)
        self.no_objects = no_objects
        masks = np.asarray(response.masks.data).reshape((no_objects, int(response.masks.layout.dim[0].size / no_objects), response.masks.layout.dim[1].size, response.masks.layout.dim[2].size)) #* 255
        self.masks = masks.astype(np.uint8)
        self.bbox = np.asarray(response.bbox.data).reshape((-1,4))
        self.objects = np.asarray(response.object.data)
        self.scores = np.asarray(response.confidence.data)

        return self.masks, self.objects, self.scores, self.bbox

    def visualizeBBox(self, img):

        if self.bbox is None:
            print("No bounding boxes to visualize")
            return 0
        if self.bbox.shape[0] < 0:
            print("No bounding boxes to visualize")
            return 0

        for i in range(self.bbox.shape[0]):

            x1 = int(self.bbox[i][0])
            y1 = int(self.bbox[i][1])
            w = int(int(self.bbox[i][2]) - x1)
            h = int(int(self.bbox[i][3]) - y1)

            img = cv2.rectangle(img, (x1, y1), (x1+w, y1+h), (0, 0, 255), 3)
            img = cv2.putText(img, str(self.OBJ_CLASSES[self.objects[i]]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3, 2)

        return img

    def processMasks(self, conf_threshold = 50, erode_kernel=(21,21)):
        if self.masks is None:
            print("No masks available to process")
            return 0
        if self.masks.shape[0] < 0:
            print("No masks available to process")
            return 0

        kernel = np.ones(erode_kernel, np.uint8)
        for i in range(self.masks.shape[0]):
            for j in range(self.masks[i].shape[0]):

                if j >= 1: # we do not treat j=0 (background)
                    mask = copy.deepcopy(self.masks[i,j])
                    m = np.zeros((mask.shape[0], mask.shape[1])).astype(np.uint8)
                    m[mask > conf_threshold] = 255
                    m = cv2.erode(m, kernel)
                    self.masks[i,j] = m
        return 1

    def visualizeMasks(self, img):

        if self.masks is None:
            print("No masks available to visualize")
            return 0
        if self.masks.shape[0] < 0:
            print("No masks available to visualize")
            return 0

        colors = [(0,0,205), (34,139,34), (192,192,128), (165, 42, 42), (128, 64, 128),
                (204, 102, 0), (184, 134, 11), (0, 153, 153), (0, 134, 141), (184, 0, 141)]

        full_mask = np.zeros(img.shape).astype(np.uint8)
        for i in range(self.masks.shape[0]):
            for j in range(self.masks[i].shape[0]):
                if j >= 1:
                    m = self.masks[i][j] != 0
                    full_mask[m] = colors[j]
        img = cv2.addWeighted(img, 1.0, full_mask, 0.7, 0)

        return img
