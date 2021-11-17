#!/usr/bin/env python3
import rospy
from graspGenerator.srv import *
import numpy as np
import cv2
import copy
from cameraService.cameraClient import CameraClient

class GraspingGeneratorClient(object):
    """docstring for GraspingGeneratorClient."""

    def __init__(self):

        self.grasps = None
        self.GPU = False

        self.collision_thresh = 0.01 # Collision threshold in collision detection
        self.num_view = 300 # View number
        self.score_thresh = 0.0 # Remove every grasp with scores less than threshold
        self.voxel_size = 0.2

    def start(self, GPU=False):

        self.GPU = GPU
        rospy.wait_for_service("/grasp_generator/start")
        startGraspingGeneratorService = rospy.ServiceProxy("/grasp_generator/start", startGraspingSrv)
        msg = startGraspingSrv()
        msg.data = self.GPU
        response = startGraspingGeneratorService(msg)

    def getGrasps(self):

        rospy.wait_for_service("/grasp_generator/result")
        graspGeneratorService = rospy.ServiceProxy("/grasp_generator/result", runGraspingSrv)
        msg = runGraspingSrv()
        response = graspGeneratorService(msg)

        self.grasps = response.data
        return self.grasps

    def setSettings(self, collision_thresh = 0.01, num_view = 300, score_thresh = 0.0, voxel_size = 0.2):

        self.collision_thresh = collision_thresh # Collision threshold in collision detection
        self.num_view = num_view # View number
        self.score_thresh = score_thresh # Remove every grasp with scores less than threshold
        self.voxel_size = voxel_size

        rospy.wait_for_service("/grasp_generator/set_settings")
        graspGeneratorService = rospy.ServiceProxy("/grasp_generator/set_settings", setSettingsGraspingSrv)
        msg = setSettingsGraspingSrv()

        msg.collision_thresh.data = self.collision_thresh
        msg.num_view = self.num_view
        msg.score_thresh = self.score_thresh
        msg.voxel_size = self.voxel_size

        response = graspGeneratorService(msg)

    def visualize(self):
        todo = True
