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

def cartessianToPolar(x, y, z):

    print("cart: ", round(x,2), round(y,2), round(z,2))
    polar = math.atan2(math.sqrt(x**2 + y**2), z)
    azimuth = math.atan2(y, x)
    if x <= 0:
        azimuth += math.pi
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
        ee_frame = "right_ee_link"

        i, grasp_msg = 0, 0
        while True:

            grasp = grasp_data.poses[i]
            grasp.header.frame_id = camera_frame
            if i > len(grasp_data.poses):
                break

            if i >= 1:
                break

            wPoseOrigin = copy.deepcopy(grasp)
            quaternion = (
                wPoseOrigin.pose.orientation.x,
                wPoseOrigin.pose.orientation.y,
                wPoseOrigin.pose.orientation.z,
                wPoseOrigin.pose.orientation.w)
            rotMat = quaternion_matrix(quaternion)[:3,:3]
            offset = np.array([[0.0], [0.0], [0.2]])
            offset = np.transpose(np.matmul(rotMat, offset))[0]

            wPoseOriginPos = np.array([wPoseOrigin.pose.position.x, wPoseOrigin.pose.position.y, wPoseOrigin.pose.position.z])
            wPoseOrigin.pose.position.x += -offset[0]
            wPoseOrigin.pose.position.y += -offset[1]
            wPoseOrigin.pose.position.z += -offset[2]

            wSphere = copy.deepcopy(wPoseOrigin)
            wSphere = transformFrame(tf_buffer, wSphere, camera_frame, world_frame)
            wGrasp = transformFrame(tf_buffer, grasp, camera_frame, world_frame)
            wSphere.pose.position.x += -wGrasp.pose.position.y
            wSphere.pose.position.y += -wGrasp.pose.position.y
            wSphere.pose.position.z += -wGrasp.pose.position.z
            x = wSphere.pose.position.x
            y = wSphere.pose.position.y
            z = wSphere.pose.position.z
            print(x,y,z)

            r, polarAngle, azimuthAngle = cartessianToPolar(x, y, z)

            print(r, polarAngle, azimuthAngle)
            azimuthAngleLimit = [-0.75*math.pi, -0.25*math.pi]
            polarAngleLimit = [0, 0.5*math.pi]
            """
            grasp_msg = grasp_data.poses[i]
            grasp_msg.header.stamp = rospy.Time.now()
            grasp_msg.header.frame_id = camera_frame
            pub_grasp.publish(grasp_msg)
            """
            wPoseOrigin.header.stamp = rospy.Time.now()
            wPoseOrigin.header.frame_id = wPoseOrigin.header.frame_id
            pub_waypoint.publish(wPoseOrigin)
            """
            if polarAngle >= polarAngleLimit[0] and polarAngle <= polarAngleLimit[1]:
                if azimuthAngle >= azimuthAngleLimit[0] and azimuthAngle <= azimuthAngleLimit[1]:
                    grasp_msg = grasp_data.poses[i]
                    grasp_msg.header.stamp = rospy.Time.now()
                    grasp_msg.header.frame_id = camera_frame
                    break
            """
            i += 1

        if grasp_msg == 0:
            print("Could not find grasp with appropriate angle")
        #pub_grasp.publish(grasp_msg)
        exit()
        #"right_ee_link"

        ############################## START HERE DANIEL #######################


        # takes a long time to run
        # make sure affordance-ros docker image is running
        #getAffordanceResult()

if __name__ == "__main__":
    main()
