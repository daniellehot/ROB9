#!/usr/bin/env python
import sys
import rospy
import std_msgs.msg
from affordancenet_service.srv import *
from realsense_service.srv import *
import numpy as np
import cv2
from std_msgs.msg import Bool
from nav_msgs.msg import Path
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import copy
import math

def captureNewScene():
    rospy.wait_for_service("/sensors/realsense/capture")
    captureService = rospy.ServiceProxy("/sensors/realsense/capture", capture)
    msg = capture()
    msg.data = True
    response = captureService(msg)
    print(response)

def getAffordanceResult():
    rospy.wait_for_service("/affordanceNet/result")
    affordanceNetService = rospy.ServiceProxy("/affordanceNet/result", affordance)
    msg = affordance()
    msg.data = True
    response = affordanceNetService(msg)

    no_objects = int(response.masks.layout.dim[0].size / 10)
    masks = np.asarray(response.masks.data).reshape((no_objects, int(response.masks.layout.dim[0].size / no_objects), response.masks.layout.dim[1].size, response.masks.layout.dim[2].size)) #* 255
    masks = masks.astype(np.uint8)

    bbox = np.asarray(response.bbox.data).reshape((-1,4))

    objects = np.asarray(response.object.data)
    print(masks.shape)
    print(masks.dtype)
    print(bbox)
    print(objects)
    for j in range(3):
        for i in range(10):
            print(j,i)
            cv2.imshow("title", masks[j][i])
            cv2.waitKey(0)


def grasp_callback(data):
    global new_grasps, grasp_data
    print('Recieved grasps')
    grasp_data = data
    new_grasps = True


def run_graspnet(pub):
    print('Send start to graspnet')
    graspnet_msg = Bool()
    graspnet_msg.data = True
    pub.publish(graspnet_msg)

def transformFrame(tf_buffer, pose, orignalFrame, newFrame):

    transformed_pose_msg = geometry_msgs.msg.PoseStamped()
    tf_buffer.lookup_transform(orignalFrame, newFrame, rospy.Time.now(), rospy.Duration(1.0))
    transformed_pose_msg = tf_buffer.transform(pose, newFrame)
    return transformed_pose_msg

def cartesianToSpherical(x, y, z):

    polar = math.atan2(math.sqrt(x**2 + y**2), z)
    azimuth = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2 + z**2)

    return r, polar, azimuth


def main():
    global new_grasps, grasp_data
    if not rospy.is_shutdown():
        rospy.init_node('grasp_pose', anonymous=True)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        rospy.Subscriber("grasps", Path, grasp_callback)
        pub_graspnet = rospy.Publisher('start_graspnet', Bool, queue_size=10)
        pub_grasp = rospy.Publisher('pose_to_reach', PoseStamped, queue_size=10)
        pub_waypoint = rospy.Publisher('pose_to_reach_waypoint', PoseStamped, queue_size=10)
        rate = rospy.Rate(5)

        # only capture a new scene at startup
        captureNewScene()

        #make graspnet run on images from realsense
        run_graspnet(pub_graspnet)

        new_grasps = False
        print('Waiting for grasps from graspnet...')
        while not new_grasps and not rospy.is_shutdown():
            rate.sleep()

        # Evaluating the best grasp.
        world_frame = "world"
        camera_frame = "ptu_camera_color_optical_frame"
        camera_frame2 = "ptu_camera_color_optical_frame_real"
        camera_frame = camera_frame2
        ee_frame = "right_ee_link"

        i, grasps, waypoints = 0, [], []
        while True:

            if i >= len(grasp_data.poses):
                break

            grasp = grasp_data.poses[i]
            grasp.header.frame_id = camera_frame


            graspCamera = copy.deepcopy(grasp)
            waypointCamera = copy.deepcopy(grasp)

            # computing waypoint in camera frame
            quaternion = (
                graspCamera.pose.orientation.x,
                graspCamera.pose.orientation.y,
                graspCamera.pose.orientation.z,
                graspCamera.pose.orientation.w)

            rotMat = quaternion_matrix(quaternion)[:3,:3]
            offset = np.array([[0.2], [0.0], [0.0]])
            offset = np.transpose(np.matmul(rotMat, offset))[0]

            waypointCamera.pose.position.x += -offset[0]
            waypointCamera.pose.position.y += -offset[1]
            waypointCamera.pose.position.z += -offset[2]

            # computing waypoint and grasp in world frame

            waypointWorld = transformFrame(tf_buffer, waypointCamera, camera_frame, world_frame)
            graspWorld = transformFrame(tf_buffer, graspCamera, camera_frame, world_frame)

            # computing local cartesian coordinates
            x = waypointWorld.pose.position.x - graspWorld.pose.position.x
            y = waypointWorld.pose.position.y - graspWorld.pose.position.y
            z = waypointWorld.pose.position.z - graspWorld.pose.position.z

            # computing spherical coordinates
            r, polarAngle, azimuthAngle = cartesianToSpherical(x, y, z)

            # Evaluating angle limits
            azimuthAngleLimit = [-0.5*math.pi, -0.25*math.pi]
            polarAngleLimit = [0, 0.5*math.pi]

            if azimuthAngle > azimuthAngleLimit[0] and azimuthAngle < azimuthAngleLimit[1]:
                if polarAngle > polarAngleLimit[0] and polarAngle < polarAngleLimit[1]:

                    waypointWorld.header.stamp = rospy.Time.now()
                    waypointWorld.header.frame_id = world_frame
                    graspWorld.header.stamp = rospy.Time.now()
                    graspWorld.header.frame_id = world_frame

                    waypoints.append(waypointWorld)
                    graspMsg.append(graspWorld)

            i += 1

        if len(grasps) == 0 or len(waypoints) == 0:
            print("Could not find grasp with appropriate angle")
        else:
            pub_waypoint.publish(waypoints[0])
            pub_grasp.publish(grasps[0])
        exit()
        #"right_ee_link"

        ############################## START HERE DANIEL #######################
        add_waypoint(grasp_msg)

if __name__ == "__main__":
    main()
