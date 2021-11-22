import rospy
import geometry_msgs
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
import tf_conversions
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from rob9.srv import tf2TransformPoseStampedSrv, tf2TransformPoseStampedSrvResponse

import math

def transformToFrame(pose, newFrame):
    """ input:  pose - geometry_msgs.PoseStamped()
                newFrame - desired frame for pose to be transformed into.
        output: transformed_pose_msg - pose in newFrame """

    pose.header.stamp = rospy.Time.now()

    rospy.wait_for_service("/tf2/transformPoseStamped")
    tf2Service = rospy.ServiceProxy("/tf2/transformPoseStamped", tf2TransformPoseStampedSrv)

    response = tf2Service(pose, String(newFrame))

    return response


def cartesianToSpherical(x, y, z):
    """ input: cartesian coordinates
        output: 3 spherical coordinates """

    polar = math.atan2(math.sqrt(x**2 + y**2), z)
    azimuth = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2 + z**2)
    return r, polar, azimuth


def delta_orientation(pose1, pose2):
    """ input: 2 posestamped ros message
        output: the difference in rotation expressed as a quaternion """

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

    deltaQuaternion = quaternion_multiply(q1, q2Inv)
    deltaRPY = np.asarray(euler_from_quaternion(deltaQuaternion)) * 180 / math.pi
    return deltaRPY
