#!/usr/bin/env python3
import rospy
import geometry_msgs
import numpy as np
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from std_msgs.msg import String
from rob9.srv import tf2TransformPoseStampedSrv, tf2TransformPoseStampedSrvResponse
from rob9.srv import tf2TransformPathSrv, tf2TransformPathSrvResponse
from rob9.srv import tf2QuatToRotSrv, tf2QuatToRotSrvResponse

import math

def transformToFrame(pose, newFrame):
    """ input:  pose - geometry_msgs.PoseStamped()
                newFrame - desired frame for pose to be transformed into.
        output: transformed_pose_msg - pose in newFrame """

    pose.header.stamp = rospy.Time.now()

    rospy.wait_for_service("/tf2/transformPoseStamped")
    tf2Service = rospy.ServiceProxy("/tf2/transformPoseStamped", tf2TransformPoseStampedSrv)

    response = tf2Service(pose, String(newFrame)).data

    return response

def transformToFramePath(path, newFrame):
    """ input:  pose - nav_msgs.Path()
            newFrame - desired frame for pose to be transformed into.
    output: transformed_path_msg - path in newFrame """

    pose.header.stamp = rospy.Time.now()

    rospy.wait_for_service("/tf2/transformPath")
    tf2Service = rospy.ServiceProxy("/tf2/transformPath", tf2TransformPathSrv)

    response = tf2Service(path, String(newFrame))

    print(response)

    return response


def quatToRot(q):

    rospy.wait_for_service("/tf2/quatToRot")
    tf2Service = rospy.ServiceProxy("/tf2/quatToRot", tf2QuatToRotSrv)

    msg = Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]

    response = tf2Service(msg).rotation.data
    R = np.reshape(np.array(response), (3,3))

    return R


def cartesianToSpherical(x, y, z):
    """ input: cartesian coordinates
        output: 3 spherical coordinates """

    polar = math.atan2(math.sqrt(x**2 + y**2), z)
    azimuth = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2 + z**2)
    return r, polar, azimuth

def quaternionMultiply(q1, q2):
    # q1
    x0, y0, z0, w0 = q1[0], q1[1], q1[2], q1[3]

    # q2
    x1, y1, z1, w1 = q2[0], q2[1], q2[2], q2[3]

    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([Q0Q1_x, Q0Q1_y, Q0Q1_z, Q0Q1_w])
    return final_quaternion

def eulerFromQuaternion(q):
    pass
"""
def delta_orientation(pose1, pose2):
    # input: 2 posestamped ros message
    #    output: the difference in rotation expressed as a quaternion

    q1 = (
        pose1.pose.orientation.x,
        pose1.pose.orientation.y,
        pose1.pose.orientation.z,
        pose1.pose.orientation.w)
    rpy1 = np.asarray(euler_from_quaternion(q1)) * 180 / math.pi

    q2Inv = (
        pose2.transform.rotation.x,
        pose2.transform.rotation.y,
        pose2.transform.rotation.z,
        -pose2.transform.rotation.w)

    deltaQuaternion = quaternionMultiply(q1, q2Inv)
    #deltaRPY = np.asarray(euler_from_quaternion(deltaQuaternion)) * 180 / math.pi
    return deltaRPY
"""
if __name__ == '__main__':
    print("Start")
    quatToRot([0, 0, 0, 1])
