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
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_multiply
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


def add_waypoint(grasp, pub):
    # Contruct a waypoint, send a message
    # print "add waypoint grasp \n", grasp
    wPoseOrigin = copy.deepcopy(grasp)
    quaternion = (
        wPoseOrigin.pose.orientation.x,
        wPoseOrigin.pose.orientation.y,
        wPoseOrigin.pose.orientation.z,
        wPoseOrigin.pose.orientation.w)
    rotMat = quaternion_matrix(quaternion)[:3,:3]
    offset = np.array([[0.0], [0.0], [-0.2]])
    offset = np.transpose(np.matmul(rotMat, offset))[0]
    # print "add waypoiny offset \n", offset

    #wPoseOriginPos = np.array([wPoseOrigin.pose.position.x, wPoseOrigin.pose.position.y, wPoseOrigin.pose.position.z])
    wPoseOrigin.pose.position.x += -offset[0]
    wPoseOrigin.pose.position.y += -offset[1]
    wPoseOrigin.pose.position.z += -offset[2]
    wPoseOrigin.header.stamp = rospy.Time.now()
    # print "add waypoint wPoseOrigin \n", wPoseOrigin
    pub.publish(wPoseOrigin)

def calculate_delta_orientation(tf_buffer, grasp_world):
    grasp_world_q = (
        grasp_world.pose.orientation.x,
        grasp_world.pose.orientation.y,
        grasp_world.pose.orientation.z,
        grasp_world.pose.orientation.w)
    grasp_world_rpy = euler_from_quaternion(grasp_world_q)
    grasp_world_rpy = np.asarray(grasp_world_rpy)
    grasp_world_rpy = grasp_world_rpy*180/3.14159265359
    #print(grasp_world_rpy)

    ee_transformation=tf_buffer.lookup_transform("world", "right_ee_link", rospy.Time.now(), rospy.Duration(1.0))
    ee_q =(
        ee_transformation.transform.rotation.x,
        ee_transformation.transform.rotation.y,
        ee_transformation.transform.rotation.z,
        ee_transformation.transform.rotation.w)
    ee_rpy = euler_from_quaternion(ee_q)
    ee_rpy = np.asarray(ee_rpy)
    ee_rpy = ee_rpy*180/3.14159265359
    #print(ee_rpy)


    ee_q_inv = (
        ee_transformation.transform.rotation.x,
        ee_transformation.transform.rotation.y,
        ee_transformation.transform.rotation.z,
        -ee_transformation.transform.rotation.w)
    qr=quaternion_multiply(grasp_world_q, ee_q_inv)
    qr_rpy = euler_from_quaternion(qr)
    qr_rpy = np.asarray(qr_rpy)
    qr_rpy = qr_rpy*180/3.14159265359
    #print(qr_rpy)
    return qr_rpy


def main(demo):
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
        ee_frame = "right_ee_link"
        world_frame = "world"
        if demo == True:
            camera_frame = "ptu_camera_color_optical_frame_real"
        else:
            camera_frame = "ptu_camera_color_optical_frame"

        length_grasp_data = len(grasp_data.poses)
        good_grasps_idx = 0
        sum_list = []
        weighted_sum_list = []
        for i in range(length_grasp_data):
            grasp = grasp_data.poses[i]
            if float(grasp.header.frame_id) > 0.25:
                good_grasps_idx += 1
                grasp.header.frame_id = camera_frame
                #pub_grasp.publish(grasp)
                #rospy.sleep(2)
                grasp_world = transformFrame(tf_buffer, grasp, camera_frame, world_frame)
                rpy_delta = calculate_delta_orientation(tf_buffer, grasp_world)
                print "rpy_delta ", rpy_delta
                rpy_delta = abs(rpy_delta)
                sum_rpy_delta = sum(rpy_delta)
                weighted_sum_rpy_delta = 0.2*rpy_delta[0]+0.4*rpy_delta[1]+0.4*rpy_delta[2]
                print "sum", sum_rpy_delta
                print "weighted sum", weighted_sum_rpy_delta
                sum_list.append(sum_rpy_delta)
                weighted_sum_list.append(weighted_sum_rpy_delta)
                print "--------------------------------------"
        #print "sum_list", sum_list
        min_sum = min(sum_list)
        #print "min_sum", min_sum
        min_index = sum_list.index(min_sum)
        print "min_index", min_index
        print "--------------------------------------"
        grasp = grasp_data.poses[min_index]
        #print "main \n", grasp
        pub_grasp.publish(grasp)

        min_sum = min(weighted_sum_list)
        #print "min_sum", min_sum
        min_index = weighted_sum_list.index(min_sum)
        print "weighted min_index", min_index
        print "--------------------------------------"
        grasp = grasp_data.poses[min_index]
        #print "main \n", grasp
        pub_waypoint.publish(grasp)
        #add_waypoint(grasp, pub_waypoint)
        #print "published the good grasp"
        exit()









        i, grasp_msg = 0, 0
        while False:
            grasp = grasp_data.poses[i]
            grasp.header.frame_id = camera_frame
            if i > len(grasp_data.poses):
                break

            if i >= 1:
                break

            grasp_world = transformFrame(tf_buffer, grasp, camera_frame, world_frame)
            grasp_world_q = (
                grasp_world.pose.orientation.x,
                grasp_world.pose.orientation.y,
                grasp_world.pose.orientation.z,
                grasp_world.pose.orientation.w)
            grasp_world_rpy = euler_from_quaternion(grasp_world_q)
            grasp_world_rpy = np.asarray(grasp_world_rpy)
            grasp_world_rpy = grasp_world_rpy*180/3.14159265359
            print(grasp_world_rpy)

            ee_transformation=tf_buffer.lookup_transform("world", "right_ee_link", rospy.Time.now(), rospy.Duration(1.0))
            ee_q =(
                ee_transformation.transform.rotation.x,
                ee_transformation.transform.rotation.y,
                ee_transformation.transform.rotation.z,
                ee_transformation.transform.rotation.w)
            ee_rpy = euler_from_quaternion(ee_q)
            ee_rpy = np.asarray(ee_rpy)
            ee_rpy = ee_rpy*180/3.14159265359
            print(ee_rpy)


            ee_q_inv = (
                ee_transformation.transform.rotation.x,
                ee_transformation.transform.rotation.y,
                ee_transformation.transform.rotation.z,
                -ee_transformation.transform.rotation.w)
            qr=quaternion_multiply(grasp_world_q, ee_q_inv)
            qr_rpy = euler_from_quaternion(qr)
            qr_rpy = np.asarray(qr_rpy)
            qr_rpy = qr_rpy*180/3.14159265359
            print(qr_rpy)


            pub_grasp.publish(grasp)




            #add_waypoint(grasp, pub_waypoint)


            """
            waypoint_world = transformFrame(tf_buffer, wPoseOrigin, camera_frame, world_frame)
            waypoint_world_q = (
                waypoint_world.pose.orientation.x,
                waypoint_world.pose.orientation.y,
                waypoint_world.pose.orientation.z,
                waypoint_world.pose.orientation.w)
            """















            """
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

            """
            grasp_msg = grasp_data.poses[i]
            grasp_msg.header.stamp = rospy.Time.now()
            grasp_msg.header.frame_id = camera_frame
            pub_grasp.publish(grasp_msg)
            """

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

if __name__ == "__main__":
    demo = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            demo = True
        else:
            print("Invalid input argument")
    main(demo)
