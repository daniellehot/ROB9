#!/usr/bin/env python3
import rospy
from graspGenerator.srv import *
import numpy as np
import cv2
from cameraService.cameraClient import CameraClient
from graspGenerator.client import GraspingGeneratorClient

if __name__ == "__main__":
    print("Usage example of client server service")

    graspClient = GraspingGeneratorClient()

    # if you want to change the default grasping settings do so before starting
    # the network or running the grasping generator

    collision_thresh = 0.01 # Collision threshold in collision detection
    num_view = 300 # View number
    score_thresh = 0.0 # Remove every grasp with scores less than threshold
    voxel_size = 0.2

    graspClient.setSettings(collision_thresh, num_view, score_thresh, voxel_size)

    # Load the network with GPU (True or False) or CPU
    graspClient.start(GPU=False)

    grasps = graspClient.getGrasps()

    print("I got ", len(grasps), " grasps")
