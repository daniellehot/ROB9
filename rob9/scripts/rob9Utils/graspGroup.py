#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Pose, Pose
from std_msgs.msg import String, Float32, Int32, Header
from rob9.msg import GraspMsg, GraspGroupMsg
from rob9.srv import graspGroupSrv, graspGroupSrvResponse
from rob9Utils.grasp import Grasp

class GraspGroup(object):
    """docstring for GraspGroup."""

    def __init__(self, grasps = []):

        if type(grasps) is not list:
            print("Only accepts grasps as list")
            return None
        self.grasps = grasps

    def __getitem__(self, index):
        return self.grasps[index]

    def __setitem__(self, index, item):
        self.grasps[index] = item

    def __len__(self):
        return len(self.grasps)

    def add(self, grasp):
        self.grasps.append(grasp)

    def combine(self, other):

        for grasp in other.grasps:
            self.add(grasp)

        return self

    def fromGraspGroupMsg(self, msg):

        self.__init__()
        for graspMsg in msg.data.grasps:
            self.add(Grasp().fromGraspMsg(graspMsg))

        return self

    def getgraspsByAffordanceLabel(self, label):

        grasps = []
        for grasp in self.grasps:
            if grasp.affordance == label:
                grasps.append(grasp)

        return grasps

    def getGraspsByFrame(self, frame):

        grasps = []
        for grasp in self.grasps:
            if grasp.frame_id == frame:
                grasps.append(grasp)

        return grasps

    def getGraspsByInstance(self, instance):

        grasps = []
        for grasp in self.grasps:
            if grasp.object_instance == instance:
                grasps.append(grasp)

        return grasps

    def getgraspsByTool(self, id):

        grasps = []
        for grasp in self.grasps:
            if grasp.tool_id == id:
                grasps.append(grasp)

        return grasps

    def setAffordanceID(self, id):

        for grasp in self.grasps:
            grasp.affordance_id = id

    def setFrameId(self, id):

        for grasp in self.grasps:
            grasp.frame_id = str(id)

    def setObjectInstance(self, instance):

        for grasp in self.grasps:
            grasp.object_instance = instance

    def setToolId(self, id):

        for grasp in self.grasps:
            grasp.tool_id = id

    def thresholdByScore(self, thresh):

        thresholdGrasps = []
        for grasp in self.grasps:
            if grasp.score >= thresh:
                thresholdGrasps.append(grasp)
        self.grasps = thresholdGrasps

    def toGraspGroupMsg(self):
        """ returns a graspGroup message """

        msg = GraspGroupMsg()
        graspList = []

        for grasp in self.grasps:
            graspList.append(grasp.toGraspMsg())

        msg.grasps = graspList
        return msg

    def toGraspGroupSrv(self):
        """ returns a graspGroup message """

        msg = graspGroupSrvResponse()
        graspList = []

        for grasp in self.grasps:
            graspList.append(grasp.toGraspMsg())

        msg.grasps = graspList
        return msg

    def transformToFrame(self, tf_buffer, frame_dest):

        grasps = []
        for grasp in self.grasps:
            grasp.transformToFrame(tf_buffer, frame_dest)

if __name__ == '__main__':
    rospy.init_node('rob9_graspgroup_test', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1)

    graspGroup = GraspGroup()
    for i in range(3):
        g = Grasp()
        graspGroup.add(g)
        g.object_instance = i
        g.affordance = i
        g.tool_id = i
        print(g.object_instance)

    g = Grasp()
    msg = graspGroup.toGraspGroupMsg()

    graspGroup.transformToFrame(tf_buffer, "world")
    msg = graspGroup.toGraspGroupMsg()

    grasps = graspGroup.getGraspsByInstance(instance = 1)
    print(grasps)
    print(graspGroup.getgraspsByTool(2))
    print(graspGroup.getgraspsByAffordanceLabel(3))
    print(graspGroup.getgraspsByFrame("world"))
