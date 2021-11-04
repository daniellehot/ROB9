#!/usr/bin/env python
import sys
import rospy
import std_msgs.msg
from moveit_scripts.srv import *
from moveit_scripts.msg import *
from grasp_pose.srv import *

def client():
    print("client")
    demo = std_msgs.msg.Bool()
    demo.data = True
    rospy.wait_for_service('get_grasps')
    rospy.wait_for_service('get_trajectories')
    #try:
    get_grasps = rospy.ServiceProxy('get_grasps', GetGrasps)
    get_trajectories = rospy.ServiceProxy('get_trajectories', GetTrajectories)
    resp_grasps = get_grasps(demo)
    #print("I have received a grasp server response \n"  + str(resp_grasps.grasps))
    resp_trajectories = get_trajectories(resp_grasps.grasps)
    #print("I have received a trajectory server response \n"  + str(resp_trajectories.trajectories.trajectories))
    print("I have received a trajectory server response \n"  + str(len(resp_trajectories.trajectories.trajectories)))
    print("I have received a trajectory server response \n"  + str(len(resp_trajectories.trajectories_poses.poses)))
    #print("response length, should be 2, is " + str(len(resp_trajectories)))
    #return resp_trajectories
    #except rospy.ServiceException as e:
        #print("Service call failed: %s"%e)
        #pass


if __name__ == "__main__":
    client()
