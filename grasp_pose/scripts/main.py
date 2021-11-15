#!/usr/bin/env python
import sys
import rospy
import std_msgs.msg
from affordancenet_service.srv import *
from realsense_service.srv import *
from grasp_pose.srv import *
import numpy as np
import cv2
import std_msgs.msg
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
# from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_multiply
import copy
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


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
    graspnet_msg = std_msgs.msg.Bool()
    graspnet_msg.data = True
    print(graspnet_msg)
    pub.publish(graspnet_msg)
    print("Done")


def transformFrame(tf_buffer, pose, orignalFrame, newFrame):
    pose.header.stamp = rospy.Time.now()
    transformed_pose_msg = geometry_msgs.msg.PoseStamped()
    tf_buffer.lookup_transform(orignalFrame, newFrame, rospy.Time.now(), rospy.Duration(1))
    transformed_pose_msg = tf_buffer.transform(pose, newFrame)
    return transformed_pose_msg

def cartesianToSpherical(x, y, z):
    polar = math.atan2(math.sqrt(x**2 + y**2), z)
    azimuth = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2 + z**2)
    return r, polar, azimuth

def calculate_delta_orientation(graspWorld, eeWorld):
    graspWorldQuaternion = (
        graspWorld.pose.orientation.x,
        graspWorld.pose.orientation.y,
        graspWorld.pose.orientation.z,
        graspWorld.pose.orientation.w)
    graspWorldRPY = np.asarray(euler_from_quaternion(graspWorldQuaternion)) * 180 / math.pi

    eeWorldQuaternionInv = (
        eeWorld.transform.rotation.x,
        eeWorld.transform.rotation.y,
        eeWorld.transform.rotation.z,
        -eeWorld.transform.rotation.w)

    deltaQuaternion = quaternion_multiply(graspWorldQuaternion, eeWorldQuaternionInv)
    deltaRPY = np.asarray(euler_from_quaternion(deltaQuaternion)) * 180 / math.pi
    return deltaRPY


def handle_get_grasps(req):
    print("handle_get_grasps")
    global new_grasps, grasp_data
    if not rospy.is_shutdown():


        # only capture a new scene at startup
        captureNewScene()

        #make graspnet run on images from realsense
        run_graspnet(pub_graspnet)

        new_grasps = False
        rate = rospy.Rate(5)
        print('Waiting for grasps from graspnet...')
        while not new_grasps and not rospy.is_shutdown():
            rate.sleep()

        # Evaluating the best grasp.
        world_frame = "world"
        ee_frame = "right_ee_link"
        if req.demo.data == True:
            camera_frame = "ptu_camera_color_optical_frame_real"
        else:
            camera_frame = "ptu_camera_color_optical_frame"

        graspData = []

        for i in range(len(grasp_data.poses)):
            grasp = grasp_data.poses[i]
            if float(grasp.header.frame_id) > 0.2:
                graspData.append(grasp)


        i, grasps, waypoints = 0, [], []
        while True:

            if i >= len(graspData):
                break
            grasp = graspData[i]
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
            offset = np.array([[0.1], [0.0], [0.0]])
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
            polarAngleLimit = [0, 0.35*math.pi]

            #azimuthAngleLimit = [-1*math.pi, 1*math.pi]
            #polarAngleLimit = [0, 0.5*math.pi]

            if azimuthAngle > azimuthAngleLimit[0] and azimuthAngle < azimuthAngleLimit[1]:
                if polarAngle > polarAngleLimit[0] and polarAngle < polarAngleLimit[1]:

                    waypointWorld.header.stamp = rospy.Time.now()
                    waypointWorld.header.frame_id = world_frame
                    graspWorld.header.stamp = rospy.Time.now()
                    graspWorld.header.frame_id = world_frame

                    waypoints.append(waypointWorld)
                    grasps.append(graspWorld)

                    #pub_waypoint.publish(waypoints[-1])
                    #pub_grasp.publish(grasps[-1])
                    #rospy.sleep(1)

            i += 1

        if len(grasps) == 0 or len(waypoints) == 0:
            print("Could not find grasp with appropriate angle")
        else:
            eeWorld=tf_buffer.lookup_transform("world", "right_ee_link", rospy.Time.now(), rospy.Duration(1.0))
            weightedSums = []
            for i in range(len(grasps)):
                deltaRPY = abs(calculate_delta_orientation(grasps[i], eeWorld))
                weightedSum = 0.2*deltaRPY[0]+0.4*deltaRPY[1]+0.4*deltaRPY[2]
                weightedSums.append(weightedSum)

            weightedSums_sorted = sorted(weightedSums)
            grasps_sorted = [None]*len(grasps)
            for i in range(len(weightedSums)):
                num = weightedSums_sorted[i]
                index=weightedSums.index(num)
                grasps_sorted[i] = grasps[index]
            # publish both the waypoint and the grasp to their own topics for visualisation
            #pub_waypoint.publish(waypoints[0])
            #pub_grasp.publish(grasps[0])
            # now publish both as a single message for moveit
            grasps_msg = nav_msgs.msg.Path()
            grasps_msg.header.frame_id = "world"
            grasps_msg.header.stamp = rospy.Time.now()
            for i in range(len(grasps)):
                if (i % 2) == 0:
                    index = 2
                else:
                    index = 1
                grasps_msg.poses.append(waypoints[i])
                grasps_msg.poses[-1].header.frame_id= str(index)
                grasps_msg.poses.append(grasps[i])
                grasps_msg.poses[-1].header.frame_id= str(index)
            print("Sent grasps")
            return grasps_msg

            #exit()


        # Affordance segmentation here

        """
        # Finding the grasp with the least angle difference
        eeWorld=tf_buffer.lookup_transform("world", "right_ee_link", rospy.Time.now(), rospy.Duration(1.0))

        weightedSums = []

        for i in range(len(grasps)):

            deltaRPY = abs(calculate_delta_orientation(graspWorld, eeWorld))
            weightedSum = 0.2*deltaRPY[0]+0.4*deltaRPY[1]+0.4*deltaRPY[2]
            weightedSums.append(weightedSum)

        minIndex = weightedSums.index(min(weightedSums))
        pub_waypoint.publish(waypoints[min_index])
        pub_grasp.publish(grasps[min_index])
        """


if __name__ == "__main__":
    rospy.init_node('grasp_pose', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("grasps", Path, grasp_callback)
    pub_graspnet = rospy.Publisher('start_graspnet', Bool, queue_size=10)
    pub_grasp = rospy.Publisher('pose_to_reach', PoseStamped, queue_size=10)
    pub_poses = rospy.Publisher('poses_to_reach', PoseArray, queue_size=10)
    pub_waypoint = rospy.Publisher('pose_to_reach_waypoint', PoseStamped, queue_size=10)

    grasp_server = rospy.Service('get_grasps', GetGrasps, handle_get_grasps)
    print("grasps server is ready")
    run_graspnet(pub_graspnet)

    rospy.spin()
    """
    demo = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            demo = True
        else:
            print("Invalid input argument")
            exit()
    """
