#!/usr/bin/env python
import sys
import copy
import rospy
import math
import numpy as np
import time

#import moveit_commander
#import moveit_msgs

import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import std_msgs.msg
from std_msgs.msg import Int8

from rob9.srv import graspGroupSrv, graspGroupSrvResponse
import rob9Utils.transformations as transform
from rob9Utils.graspGroup import GraspGroup
from rob9Utils.grasp import Grasp
import rob9Utils.moveit as moveit

from moveit_scripts.srv import *
from moveit_scripts.msg import *
from grasp_aff_association.srv import *

#from rob9Utils.visualize import visualizeGrasps6DOF
# from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

def send_trajectory_to_rviz(plan):
    print("Trajectory was sent to RViZ")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory_start = moveit.getCurrentState()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)


def callback(msg):
    global resp_trajectories
    #global receivedGripperCommand
    #print("Callback")
    #print("message data " + str(msg.data))
    id = msg.data
    id = str(id)
    plans = []
    goal_poses = []
    for i in range(len(resp_trajectories.trajectories.trajectories)):
        if resp_trajectories.trajectories.trajectories[i].joint_trajectory.header.frame_id == id:
            plans.append(resp_trajectories.trajectories.trajectories[i])

    for i in range(len(resp_trajectories.trajectories_poses.poses)):
        if resp_trajectories.trajectories_poses.poses[i].header.frame_id == id:
            goal_poses.append(resp_trajectories.trajectories_poses.poses[i])

    #print("I have sampled these trajectories "  + str(len(plans)))
    #print("I have sampled these goal poses "  + str(len(goal_poses)))

    waypoint_msg = geometry_msgs.msg.PoseStamped()
    waypoint_msg.header.frame_id = "world"
    waypoint_msg.header.stamp = rospy.Time.now()
    waypoint_msg.pose = goal_poses[0].pose
    pub_waypoint.publish(waypoint_msg)

    goal_msg = geometry_msgs.msg.PoseStamped()
    goal_msg.header.frame_id = "world"
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.pose = goal_poses[1].pose
    pub_grasp.publish(goal_msg)

    #rospy.sleep(6.)
    raw_input("Press Enter when you are ready to move the robot")
    for i in range(3):
        send_trajectory_to_rviz(plans[i])
        #raw_input("Press Enter when you are ready to move the robot")
        #move_group.execute(plans[i], wait=True)
        #move_group.stop()
        #move_group.clear_pose_targets()
        moveit.execute(plans[i])
        if i == 1:
            gripper_pub.publish(close_gripper_msg)
            rospy.sleep(1)
        print("I have grasped!")
    raw_input("Press Enter when you are ready to move the robot to the handover pose")
    #move_to_handover()
    moveit.moveToNamed("ready")
    moveit.moveToNamed("handover")
    raw_input("Press Enter when you are ready to move the robot back to the ready pose")

    moveit.moveToNamed("ready")
    gripper_pub.publish(open_gripper_msg)
    #move_to_ready()

def computeWaypoints(graspObjects, offset = 0.1):
    """ input:  graspsObjects   -   GraspGroup() of grasps
                offset          -   float, in meters for waypoint in relation to grasp
        output:                 -   nav_msgs Path
    """

    world_frame = "world"
    ee_frame = "right_ee_link"

    grasps, waypoints = [], []
    for i in range(len(graspObjects)):

        grasp = graspObjects[i]

        #q_x_180 = [0, 0, 1, 0]
        #g_q = grasp.orientation.getVector(format="xyzw")
        #q = transform.quaternionMultiply(g_q, q_x_180)

        #print(grasp.orientation)

        #grasp.orientation.x = q[0]
        #grasp.orientation.y = q[1]
        #grasp.orientation.z = q[2]
        #grasp.orientation.w = q[3]

        #print(grasp.orientation)
        #print()

        graspCamera = copy.deepcopy(grasp)
        waypointCamera = copy.deepcopy(grasp)

        # computing waypoint in camera frame
        rotMat = graspCamera.getRotationMatrix()
        offsetArr = np.array([[offset], [0.0], [0.0]])
        offsetCam = np.transpose(np.matmul(rotMat, offsetArr))[0]

        waypointCamera.position.x += -offsetCam[0]
        waypointCamera.position.y += -offsetCam[1]
        waypointCamera.position.z += -offsetCam[2]

        waypointWorld = Grasp().fromPoseStampedMsg(transform.transformToFrame(waypointCamera.toPoseStampedMsg(), world_frame))
        graspWorld = Grasp().fromPoseStampedMsg(transform.transformToFrame(graspCamera.toPoseStampedMsg(), world_frame))

        waypointWorld.frame_id = str(graspObjects[i].tool_id) # we should probably do away with storing it in the header
        graspWorld.frame_id = str(graspObjects[i].tool_id)
        waypoints.append(waypointWorld.toPoseStampedMsg())
        grasps.append(graspWorld.toPoseStampedMsg())
        print(i+1, " / ", len(graspObjects))

    grasps_msg = nav_msgs.msg.Path()
    grasps_msg.header.frame_id = "world"
    grasps_msg.header.stamp = rospy.Time.now()
    for i in range(len(grasps)):
        grasps_msg.poses.append(waypoints[i])
        grasps_msg.poses.append(grasps[i])

    return grasps_msg

def filterBySphericalCoordinates(poses, azimuth, polar):
    """ input:  poses   -   nav_msgs/Path, a list of waypoints and poses in pairs
                azimut  -   numpy array shape 2, [minAngle, maxAngle]
                polar   -   numpy array shape 2, [minAngle, maxAngle]
        output:         -   nav_msgs/Path a list of waypoints and poses in pairs
        Only returns waypoints and grasps that are inside the spherical angle
        limits
    """
    grasps, waypoints = [], []
    for i in range(int(len(poses.poses) / 2)):

        waypointWorld = poses.poses[i]
        graspWorld = poses.poses[i + 1]

        # computing local cartesian coordinates
        x = waypointWorld.pose.position.x - graspWorld.pose.position.x
        y = waypointWorld.pose.position.y - graspWorld.pose.position.y
        z = waypointWorld.pose.position.z - graspWorld.pose.position.z

        # computing spherical coordinates
        r, polarAngle, azimuthAngle = transform.cartesianToSpherical(x, y, z)

        azimuthAngleLimit = azimuth
        polarAngleLimit = polar

        # Evaluating angle limits
        if azimuthAngle > azimuthAngleLimit[0] and azimuthAngle < azimuthAngleLimit[1]:
            if polarAngle > polarAngleLimit[0] and polarAngle < polarAngleLimit[1]:
                waypoints.append(waypointWorld)
                grasps.append(graspWorld)


    if len(grasps) == 0 or len(waypoints) == 0:
        print("Could not find grasp with appropriate angle")
        grasp_msg = nav_msgs.msg.Path()
        return grasps_msg # in case no grasps can be found, return empty message

    grasps_msg = nav_msgs.msg.Path()
    grasps_msg.header.frame_id = "world"
    grasps_msg.header.stamp = rospy.Time.now()
    for i in range(len(grasps)):
        grasps_msg.poses.append(waypoints[i])
        grasps_msg.poses.append(grasps[i])

    return grasps_msg

def sortByOrientationDifference(poses):
    # Should be moved to GraspGroup.py
    # not yet implemented! this is the old deltaRPY
    """ input:  poses   -   nav_msgs/Path, a list of waypoints and grasps in pairs
        output:         -   nav_msgs/Path, a list of waypoints and grasps in pairs
        Sorted by the relative angle difference compared to current robot pose
    """

    eeWorld = tf_buffer.lookup_transform("world", "right_ee_link", rospy.Time.now(), rospy.Duration(1.0))
    weightedSums = []

    for i in range(len(grasps)):
        deltaRPY = abs(transform.delta_orientation(grasps[i], eeWorld))
        weightedSum = 0.2*deltaRPY[0]+0.4*deltaRPY[1]+0.4*deltaRPY[2]
        weightedSums.append(weightedSum)

    weightedSums_sorted = sorted(weightedSums)
    grasps_sorted = [None]*len(grasps) # sorted according to delta orientation from current orientation of gripper
    waypoints_sorted = [None]*len(waypoints)

    for i in range(len(weightedSums)):
        num = weightedSums_sorted[i]
        index = weightedSums.index(num)
        grasps_sorted[i] = grasps[index]
        waypoints_sorted[i] = waypoints[index]

if __name__ == '__main__':
    demo = std_msgs.msg.Bool()
    demo.data = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            demo.data = True
            print("Demo = True")
        else:
            print("Invalid input argument")
            exit()

    print("Init")
    rospy.init_node('moveit_subscriber', anonymous=True)
    rospy.Subscriber('tool_id', Int8, callback)
    gripper_pub = rospy.Publisher('gripper_controller', Int8, queue_size=10, latch=True)
    pub_grasp = rospy.Publisher('pose_to_reach', PoseStamped, queue_size=10)
    pub_waypoint = rospy.Publisher('pose_to_reach_waypoint', PoseStamped, queue_size=10)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # DO NOT REMOVE THIS SLEEP, it allows gripper_pub to establish connection to the topic
    rospy.sleep(0.1)
    rospy.sleep(2)

    reset_gripper_msg = std_msgs.msg.Int8()
    reset_gripper_msg.data = 0
    activate_gripper_msg = std_msgs.msg.Int8()
    activate_gripper_msg.data = 1
    close_gripper_msg = std_msgs.msg.Int8()
    close_gripper_msg = 2
    open_gripper_msg = std_msgs.msg.Int8()
    open_gripper_msg.data = 3
    basic_gripper_msg = std_msgs.msg.Int8()
    basic_gripper_msg.data = 4
    pinch_gripper_msg = std_msgs.msg.Int8()
    pinch_gripper_msg.data = 5

    gripper_pub.publish(reset_gripper_msg)
    gripper_pub.publish(activate_gripper_msg)
    gripper_pub.publish(open_gripper_msg)
    gripper_pub.publish(pinch_gripper_msg)

    print("Services init")
    rospy.wait_for_service('get_grasps')
    rospy.wait_for_service('get_trajectories')
    get_grasps_service = rospy.ServiceProxy('get_grasps', graspGroupSrv)
    get_trajectories = rospy.ServiceProxy('get_trajectories', GetTrajectories)
    print("Calling the grasp service")
    grasps_affordance = GraspGroup().fromGraspGroupSrv(get_grasps_service(demo))
    grasps_affordance.sortByScore()
    grasps_affordance = GraspGroup(grasps = grasps_affordance[:50])
    grasps_affordance.thresholdByScore(0.0)
    grasp_waypoints_path = computeWaypoints(grasps_affordance, offset = 0.1)

    #cloud, cloudColor = cam.getPointCloudStatic()
    #pcd = o3d.geometry.PointCloud()
    #pcd.points = o3d.utility.Vector3dVector(cloud)
    #pcd.colors = o3d.utility.Vector3dVector(cloudColor)

    #visualizeGrasps6DOF(pcd, GraspGroup(grasps = graspObjects.fromPath(grasp_waypoints_path)))

    azimuthAngleLimit = [-1*math.pi, 1*math.pi]
    polarAngleLimit = [0, 0.5*math.pi]
    #grasp_waypoints_path = filterBySphericalCoordinates(grasp_waypoints_path, azimuth = azimuthAngleLimit, polar = polarAngleLimit)

    print("Calling the trajectory service")
    resp_trajectories = get_trajectories(grasp_waypoints_path)
    print("I have received a trajectory server response ")

    id_list_duplicates = []
    for i in range(len(resp_trajectories.trajectories.trajectories)):
        id_list_duplicates.append(resp_trajectories.trajectories.trajectories[i].joint_trajectory.header.frame_id)
    id_list = list(dict.fromkeys(id_list_duplicates))
    print("id_list_duplicates " + str(id_list_duplicates))
    print("id_list " + str(id_list))

    #print("I have received a trajectory server response "  + str(len(resp_trajectories.trajectories.trajectories)))
    #print("I have received goal poses response "  + str(len(resp_trajectories.trajectories_poses.poses)))
    #print(resp_trajectories.trajectories_poses.header)
    #print("----------------------------")
    #print("----------------------------")
    #print("----------------------------")
    #print(resp_trajectories.trajectories_poses.poses[0])


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
