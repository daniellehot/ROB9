#!/usr/bin/env python
import sys
import rospy
import numpy as np
import cv2
import copy
import math

if __name__ == "__main__":
    grasps = ["a", "b", "c", "d", "e"]
    weightedSums = [260, 120, 300, 20, 60]
    weightedSums_sorted = sorted(weightedSums)
    grasps_sorted = [None]*len(grasps)
    for i in range(len(weightedSums)):
        num = weightedSums_sorted[i]
        index=weightedSums.index(num)
        grasps_sorted[i] = grasps[index]
    print("grasp sorted " + str(grasps_sorted))
